#include "offline_map.h"

#include <math.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "ble_rtc.h"
#include "board.h"
#include "gps.h"
#include "jpeg_decoder.h"
#include "screen_control.h"
#include "sd_storage.h"

#define MAP_INDEX_PATH "/sdcard/ultrawatch/maps/map.uwi"
#define MAP_MAGIC "UWMAP001"
#define MAP_VERSION 1
#define MAP_HEADER_BYTES 76
#define MAP_ENTRY_BYTES 28
#define MAP_TILE_SIZE 256
#define MAP_TILE_PIXELS (MAP_TILE_SIZE * MAP_TILE_SIZE)
#define MAP_CACHE_TILES 16
#define MAP_PAN_STEP_PIXELS (MAP_TILE_SIZE / 2)

typedef struct {
    uint8_t zoom;
    uint32_t x;
    uint32_t y;
    uint32_t segment;
    uint64_t offset;
    uint32_t length;
} map_entry_t;

typedef struct {
    bool used;
    bool missing;
    uint8_t zoom;
    uint32_t x;
    uint32_t y;
    uint32_t age;
    uint16_t *pixels;
} map_tile_t;

static const char *TAG = "offline_map";
static TaskHandle_t map_task_handle;
static SemaphoreHandle_t map_lock;
static SemaphoreHandle_t stopped_signal;
static volatile bool stop_requested;
static uint16_t *rendered_frame;
static uint16_t *working_frame;
static offline_map_snapshot_t public_snapshot;
static FILE *index_file;
static FILE *segment_file;
static uint32_t open_segment = UINT32_MAX;
static bool sd_acquired;
static uint32_t tile_count;
static uint32_t index_entries_offset;
static uint8_t min_zoom;
static uint8_t max_zoom;
static uint32_t max_tile_bytes;
static uint32_t zoom_mask;
static double archive_center_lon;
static double archive_center_lat;
static uint8_t current_zoom;
static double center_world_x;
static double center_world_y;
static bool following = true;
static bool fix_zoom_selected;
static map_tile_t cache[MAP_CACHE_TILES];
static uint32_t cache_age;
static uint8_t *jpeg_input;
static size_t jpeg_input_size;
static uint16_t *jpeg_output;
static size_t jpeg_output_size;

static bool find_entry(uint8_t zoom, uint32_t x, uint32_t y,
                       map_entry_t *entry);

static uint32_t read_u32(const uint8_t *data)
{
    return (uint32_t)data[0] | (uint32_t)data[1] << 8 |
           (uint32_t)data[2] << 16 | (uint32_t)data[3] << 24;
}

static uint64_t read_u64(const uint8_t *data)
{
    return (uint64_t)read_u32(data) | (uint64_t)read_u32(data + 4) << 32;
}

static void set_error(const char *message)
{
    xSemaphoreTake(map_lock, portMAX_DELAY);
    public_snapshot.state = OFFLINE_MAP_ERROR;
    snprintf(public_snapshot.error, sizeof(public_snapshot.error), "%s",
             message);
    xSemaphoreGive(map_lock);
}

static void lonlat_to_world(double lon, double lat, uint8_t zoom,
                            double *x, double *y)
{
    if (lat > 85.05112878) lat = 85.05112878;
    if (lat < -85.05112878) lat = -85.05112878;
    const double scale = (double)(1ULL << zoom) * MAP_TILE_SIZE;
    const double radians = lat * M_PI / 180.0;
    *x = (lon + 180.0) / 360.0 * scale;
    *y = (1.0 - log(tan(radians) + 1.0 / cos(radians)) / M_PI) *
         0.5 * scale;
}

static uint8_t next_available_zoom(uint8_t zoom, int direction)
{
    int candidate = (int)zoom + (direction > 0 ? 1 : -1);
    while (candidate >= min_zoom && candidate <= max_zoom) {
        if ((zoom_mask & (1U << candidate)) != 0) return (uint8_t)candidate;
        candidate += direction > 0 ? 1 : -1;
    }
    return zoom;
}

static uint8_t detailed_zoom_for(double lon, double lat,
                                 double *world_x, double *world_y)
{
    for (int zoom = max_zoom; zoom >= min_zoom; zoom--) {
        if ((zoom_mask & (1U << zoom)) == 0) continue;
        lonlat_to_world(lon, lat, (uint8_t)zoom, world_x, world_y);
        map_entry_t entry;
        if (find_entry((uint8_t)zoom, (uint32_t)*world_x / MAP_TILE_SIZE,
                       (uint32_t)*world_y / MAP_TILE_SIZE, &entry)) {
            return (uint8_t)zoom;
        }
    }
    lonlat_to_world(lon, lat, min_zoom, world_x, world_y);
    return min_zoom;
}

static int compare_key(uint8_t zoom, uint32_t x, uint32_t y,
                       const map_entry_t *entry)
{
    if (zoom != entry->zoom) return zoom < entry->zoom ? -1 : 1;
    if (x != entry->x) return x < entry->x ? -1 : 1;
    if (y != entry->y) return y < entry->y ? -1 : 1;
    return 0;
}

static bool read_entry(uint32_t index, map_entry_t *entry)
{
    uint8_t raw[MAP_ENTRY_BYTES];
    uint64_t offset = index_entries_offset + (uint64_t)index * MAP_ENTRY_BYTES;
    if (offset > LONG_MAX || fseek(index_file, (long)offset, SEEK_SET) != 0 ||
        fread(raw, 1, sizeof(raw), index_file) != sizeof(raw)) {
        return false;
    }
    entry->zoom = raw[0];
    entry->x = read_u32(raw + 4);
    entry->y = read_u32(raw + 8);
    entry->segment = read_u32(raw + 12);
    entry->offset = read_u64(raw + 16);
    entry->length = read_u32(raw + 24);
    return true;
}

static bool find_entry(uint8_t zoom, uint32_t x, uint32_t y,
                       map_entry_t *entry)
{
    uint32_t low = 0;
    uint32_t high = tile_count;
    while (low < high) {
        uint32_t middle = low + (high - low) / 2;
        if (!read_entry(middle, entry)) return false;
        int comparison = compare_key(zoom, x, y, entry);
        if (comparison == 0) return true;
        if (comparison < 0) high = middle;
        else low = middle + 1;
    }
    return false;
}

static bool open_tile_segment(uint32_t segment)
{
    if (segment_file != NULL && open_segment == segment) return true;
    if (segment_file != NULL) fclose(segment_file);
    char path[48];
    snprintf(path, sizeof(path), "/sdcard/ultrawatch/maps/map.%03lu",
             (unsigned long)segment);
    segment_file = fopen(path, "rb");
    open_segment = segment_file != NULL ? segment : UINT32_MAX;
    return segment_file != NULL;
}

static map_tile_t *cache_slot(uint8_t zoom, uint32_t x, uint32_t y)
{
    map_tile_t *oldest = &cache[0];
    for (int index = 0; index < MAP_CACHE_TILES; index++) {
        if (cache[index].used && cache[index].zoom == zoom &&
            cache[index].x == x && cache[index].y == y) {
            cache[index].age = ++cache_age;
            return &cache[index];
        }
        if (!cache[index].used) oldest = &cache[index];
        else if (oldest->used && cache[index].age < oldest->age) oldest = &cache[index];
    }
    oldest->used = true;
    oldest->missing = true;
    oldest->zoom = zoom;
    oldest->x = x;
    oldest->y = y;
    oldest->age = ++cache_age;
    return oldest;
}

static map_tile_t *load_tile(uint8_t zoom, uint32_t x, uint32_t y)
{
    for (int index = 0; index < MAP_CACHE_TILES; index++) {
        if (cache[index].used && cache[index].zoom == zoom &&
            cache[index].x == x && cache[index].y == y) {
            cache[index].age = ++cache_age;
            return &cache[index];
        }
    }
    map_tile_t *slot = cache_slot(zoom, x, y);
    map_entry_t entry;
    if (!find_entry(zoom, x, y, &entry) || entry.length > jpeg_input_size ||
        entry.offset > LONG_MAX || !open_tile_segment(entry.segment) ||
        fseek(segment_file, (long)entry.offset, SEEK_SET) != 0 ||
        fread(jpeg_input, 1, entry.length, segment_file) != entry.length) {
        return slot;
    }
    esp_jpeg_image_cfg_t config = {
        .indata = jpeg_input,
        .indata_size = entry.length,
        .outbuf = (uint8_t *)jpeg_output,
        .outbuf_size = jpeg_output_size,
        .out_format = JPEG_IMAGE_FORMAT_RGB565,
        .out_scale = JPEG_IMAGE_SCALE_0,
        .flags = {.swap_color_bytes = 1},
    };
    esp_jpeg_image_output_t output = {0};
    int64_t started = esp_timer_get_time();
    if (esp_jpeg_decode(&config, &output) == ESP_OK &&
        output.width == MAP_TILE_SIZE && output.height == MAP_TILE_SIZE &&
        output.output_len >= MAP_TILE_PIXELS * sizeof(uint16_t)) {
        if (slot->pixels == NULL) {
            slot->pixels = heap_caps_malloc(MAP_TILE_PIXELS * sizeof(uint16_t),
                                            MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        }
        if (slot->pixels != NULL) {
            memcpy(slot->pixels, jpeg_output, MAP_TILE_PIXELS * sizeof(uint16_t));
            slot->missing = false;
            ESP_LOGD(TAG, "tile z%u/%lu/%lu: %lld ms", zoom,
                     (unsigned long)x, (unsigned long)y,
                     (esp_timer_get_time() - started) / 1000);
        }
    }
    return slot;
}

static void render_map(void)
{
    gps_status_t gps = {0};
    bool fix = gps_get_status(&gps) && gps.ready && gps.fix_valid;
    xSemaphoreTake(map_lock, portMAX_DELAY);
    if (fix && following) {
        const double longitude = gps.longitude_e7 / 1e7;
        const double latitude = gps.latitude_e7 / 1e7;
        if (!fix_zoom_selected) {
            current_zoom = detailed_zoom_for(longitude, latitude,
                                             &center_world_x,
                                             &center_world_y);
            fix_zoom_selected = true;
        } else {
            lonlat_to_world(longitude, latitude, current_zoom,
                            &center_world_x, &center_world_y);
        }
    }
    const double local_center_x = center_world_x;
    const double local_center_y = center_world_y;
    const uint8_t local_zoom = current_zoom;
    const bool local_following = following;
    xSemaphoreGive(map_lock);
    const int64_t center_x = (int64_t)llround(local_center_x);
    const int64_t center_y = (int64_t)llround(local_center_y);
    for (int screen_y = 0; screen_y < OFFLINE_MAP_HEIGHT; screen_y++) {
        int dy = screen_y - OFFLINE_MAP_HEIGHT / 2;
        for (int screen_x = 0; screen_x < OFFLINE_MAP_WIDTH; screen_x++) {
            int dx = screen_x - OFFLINE_MAP_WIDTH / 2;
            int64_t world_x = center_x + dx;
            int64_t world_y = center_y + dy;
            uint32_t limit = 1U << local_zoom;
            if (world_x < 0 || world_y < 0 ||
                world_x >= (int64_t)limit * MAP_TILE_SIZE ||
                world_y >= (int64_t)limit * MAP_TILE_SIZE) {
                working_frame[screen_y * OFFLINE_MAP_WIDTH + screen_x] = 0;
                continue;
            }
            uint32_t tile_x = (uint32_t)world_x / MAP_TILE_SIZE;
            uint32_t tile_y = (uint32_t)world_y / MAP_TILE_SIZE;
            map_tile_t *tile = load_tile(local_zoom, tile_x, tile_y);
            if (tile->missing || tile->pixels == NULL) {
                bool checker = ((world_x / 24) + (world_y / 24)) & 1;
                working_frame[screen_y * OFFLINE_MAP_WIDTH + screen_x] =
                    checker ? 0x0821 : 0x0000;
            } else {
                int pixel_x = (int)(world_x % MAP_TILE_SIZE);
                int pixel_y = (int)(world_y % MAP_TILE_SIZE);
                working_frame[screen_y * OFFLINE_MAP_WIDTH + screen_x] =
                    tile->pixels[pixel_y * MAP_TILE_SIZE + pixel_x];
            }
        }
    }
    xSemaphoreTake(map_lock, portMAX_DELAY);
    uint16_t *completed = rendered_frame;
    rendered_frame = working_frame;
    working_frame = completed;
    public_snapshot.state = fix ? OFFLINE_MAP_READY : OFFLINE_MAP_GPS_SEARCH;
    public_snapshot.following = local_following;
    public_snapshot.gps_fix = fix;
    public_snapshot.zoom = local_zoom;
    public_snapshot.can_zoom_in = next_available_zoom(local_zoom, 1) != local_zoom;
    public_snapshot.can_zoom_out = next_available_zoom(local_zoom, -1) != local_zoom;
    public_snapshot.marker_x = OFFLINE_MAP_WIDTH / 2;
    public_snapshot.marker_y = OFFLINE_MAP_HEIGHT / 2;
    if (fix && !local_following) {
        double live_x, live_y;
        lonlat_to_world(gps.longitude_e7 / 1e7, gps.latitude_e7 / 1e7,
                        local_zoom, &live_x, &live_y);
        double source_x = live_x - local_center_x;
        double source_y = live_y - local_center_y;
        public_snapshot.marker_x = OFFLINE_MAP_WIDTH / 2 +
            (int)llround(source_x);
        public_snapshot.marker_y = OFFLINE_MAP_HEIGHT / 2 +
            (int)llround(source_y);
    }
    xSemaphoreGive(map_lock);
}

static esp_err_t open_package(void)
{
    ESP_RETURN_ON_ERROR(sd_storage_acquire(), TAG, "SD mount failed");
    sd_acquired = true;
    if (stop_requested) return ESP_ERR_INVALID_STATE;
    index_file = fopen(MAP_INDEX_PATH, "rb");
    if (index_file == NULL) return ESP_ERR_NOT_FOUND;
    uint8_t header[MAP_HEADER_BYTES];
    if (fread(header, 1, sizeof(header), index_file) != sizeof(header) ||
        memcmp(header, MAP_MAGIC, 8) != 0 || read_u32(header + 8) != MAP_VERSION) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    tile_count = read_u32(header + 12);
    min_zoom = (uint8_t)read_u32(header + 16);
    max_zoom = (uint8_t)read_u32(header + 20);
    max_tile_bytes = read_u32(header + 28);
    zoom_mask = read_u32(header + 32);
    archive_center_lon = (int32_t)read_u32(header + 68) / 1e7;
    archive_center_lat = (int32_t)read_u32(header + 72) / 1e7;
    uint8_t length_raw[4];
    if (fread(length_raw, 1, 4, index_file) != 4) return ESP_ERR_INVALID_RESPONSE;
    uint32_t attribution_length = read_u32(length_raw);
    index_entries_offset = MAP_HEADER_BYTES + 4 + attribution_length;
    if (tile_count == 0 || min_zoom > max_zoom || max_zoom > 23 ||
        zoom_mask == 0 ||
        max_tile_bytes == 0 || fseek(index_file, attribution_length, SEEK_CUR) != 0) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    current_zoom = detailed_zoom_for(archive_center_lon, archive_center_lat,
                                     &center_world_x, &center_world_y);
    fix_zoom_selected = false;
    jpeg_input_size = max_tile_bytes;
    jpeg_input = heap_caps_malloc(jpeg_input_size,
                                  MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    jpeg_output_size = MAP_TILE_PIXELS * sizeof(uint16_t);
    jpeg_output = heap_caps_malloc(jpeg_output_size,
                                   MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    return jpeg_input != NULL && jpeg_output != NULL ? ESP_OK : ESP_ERR_NO_MEM;
}

static void close_package(void)
{
    if (segment_file != NULL) fclose(segment_file);
    segment_file = NULL;
    open_segment = UINT32_MAX;
    if (index_file != NULL) fclose(index_file);
    index_file = NULL;
    free(jpeg_input);
    jpeg_input = NULL;
    free(jpeg_output);
    jpeg_output = NULL;
    for (int index = 0; index < MAP_CACHE_TILES; index++) {
        free(cache[index].pixels);
        memset(&cache[index], 0, sizeof(cache[index]));
    }
    if (sd_acquired) sd_storage_release();
    sd_acquired = false;
}

static void map_task(void *parameter)
{
    (void)parameter;
    int64_t mount_started = esp_timer_get_time();
    esp_err_t result = open_package();
    ESP_LOGI(TAG, "map package open: %lld ms", (esp_timer_get_time() - mount_started) / 1000);
    bool resources_closed = false;
    if (result != ESP_OK) {
        close_package();
        ble_rtc_release_gps_lease();
        resources_closed = true;
        if (!stop_requested) {
            set_error(result == ESP_ERR_NOT_FOUND ? "KARTE FEHLT" : "SD FEHLER");
            screen_request_refresh();
        }
        while (!stop_requested) {
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        }
    }
    while (result == ESP_OK && !stop_requested) {
        int64_t started = esp_timer_get_time();
        render_map();
        ESP_LOGI(TAG, "map frame: %lld ms", (esp_timer_get_time() - started) / 1000);
        screen_request_refresh();
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000));
    }
    if (!resources_closed) {
        close_package();
        ble_rtc_release_gps_lease();
    }
    xSemaphoreTake(map_lock, portMAX_DELAY);
    public_snapshot.state = OFFLINE_MAP_STOPPED;
    xSemaphoreGive(map_lock);
    map_task_handle = NULL;
    xSemaphoreGive(stopped_signal);
    vTaskDelete(NULL);
}

esp_err_t offline_map_start(void)
{
    if (map_task_handle != NULL) return ESP_OK;
    if (map_lock == NULL) map_lock = xSemaphoreCreateMutex();
    if (stopped_signal == NULL) stopped_signal = xSemaphoreCreateBinary();
    if (map_lock == NULL || stopped_signal == NULL) return ESP_ERR_NO_MEM;
    if (rendered_frame == NULL) {
        rendered_frame = heap_caps_calloc(OFFLINE_MAP_WIDTH * OFFLINE_MAP_HEIGHT,
                                          sizeof(uint16_t),
                                          MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (rendered_frame == NULL) return ESP_ERR_NO_MEM;
    }
    if (working_frame == NULL) {
        working_frame = heap_caps_calloc(OFFLINE_MAP_WIDTH * OFFLINE_MAP_HEIGHT,
                                         sizeof(uint16_t),
                                         MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (working_frame == NULL) return ESP_ERR_NO_MEM;
    }
    xSemaphoreTake(map_lock, portMAX_DELAY);
    memset(&public_snapshot, 0, sizeof(public_snapshot));
    public_snapshot.state = OFFLINE_MAP_LOADING;
    public_snapshot.following = true;
    xSemaphoreGive(map_lock);
    following = true;
    stop_requested = false;
    xSemaphoreTake(stopped_signal, 0);
    ESP_RETURN_ON_ERROR(ble_rtc_acquire_gps_lease(), TAG, "GPS lease failed");
    if (xTaskCreate(map_task, "offline_map", 6144, NULL, 3,
                    &map_task_handle) != pdPASS) {
        ble_rtc_release_gps_lease();
        map_task_handle = NULL;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

void offline_map_stop(void)
{
    if (map_task_handle == NULL) return;
    stop_requested = true;
    xTaskNotifyGive(map_task_handle);
    xSemaphoreTake(stopped_signal, portMAX_DELAY);
}

bool offline_map_is_active(void)
{
    return map_task_handle != NULL;
}

void offline_map_pan(int screen_dx, int screen_dy)
{
    if (map_task_handle == NULL) return;
    xSemaphoreTake(map_lock, portMAX_DELAY);
    center_world_x -= screen_dx;
    center_world_y -= screen_dy;
    following = false;
    xSemaphoreGive(map_lock);
    xTaskNotifyGive(map_task_handle);
}

void offline_map_pan_step(int direction_x, int direction_y)
{
    if (map_task_handle == NULL ||
        (direction_x == 0 && direction_y == 0)) return;
    xSemaphoreTake(map_lock, portMAX_DELAY);
    center_world_x += direction_x * MAP_PAN_STEP_PIXELS;
    center_world_y += direction_y * MAP_PAN_STEP_PIXELS;
    following = false;
    xSemaphoreGive(map_lock);
    xTaskNotifyGive(map_task_handle);
}

void offline_map_zoom(int direction)
{
    if (map_task_handle == NULL || direction == 0) return;
    xSemaphoreTake(map_lock, portMAX_DELAY);
    uint8_t selected = next_available_zoom(current_zoom, direction);
    if (selected != current_zoom) {
        center_world_x = ldexp(center_world_x, (int)selected - current_zoom);
        center_world_y = ldexp(center_world_y, (int)selected - current_zoom);
        current_zoom = selected;
    }
    xSemaphoreGive(map_lock);
    xTaskNotifyGive(map_task_handle);
}

void offline_map_recenter(void)
{
    if (map_task_handle == NULL) return;
    xSemaphoreTake(map_lock, portMAX_DELAY);
    following = true;
    xSemaphoreGive(map_lock);
    xTaskNotifyGive(map_task_handle);
}

bool offline_map_copy_frame(uint16_t *destination,
                            offline_map_snapshot_t *snapshot)
{
    if (destination == NULL || snapshot == NULL || map_lock == NULL ||
        rendered_frame == NULL) return false;
    xSemaphoreTake(map_lock, portMAX_DELAY);
    memcpy(destination, rendered_frame,
           OFFLINE_MAP_WIDTH * OFFLINE_MAP_HEIGHT * sizeof(uint16_t));
    *snapshot = public_snapshot;
    xSemaphoreGive(map_lock);
    return true;
}
