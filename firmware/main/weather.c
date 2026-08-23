#include "weather.h"

#include <ctype.h>
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

#include "ble_rtc.h"
#include "cJSON.h"
#include "esp_crt_bundle.h"
#include "esp_heap_caps.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "gps.h"
#include "screen_control.h"
#include "sd_storage.h"
#include "wifi_manager.h"

#define WEATHER_CONFIG_PATH "/sdcard/ultrawatch/config.txt"
#define WEATHER_CACHE_DIRECTORY "/sdcard/ultrawatch/weather"
#define WEATHER_CACHE_PATH WEATHER_CACHE_DIRECTORY "/cache.json"
#define WEATHER_CACHE_TMP_PATH WEATHER_CACHE_DIRECTORY "/cache.tmp"
#define WEATHER_CACHE_BACKUP_PATH WEATHER_CACHE_DIRECTORY "/cache.bak"
#define WEATHER_CACHE_REFRESH_SECONDS 3600

static const char *TAG = "weather";
static SemaphoreHandle_t weather_lock;
static TaskHandle_t weather_task_handle;
static weather_snapshot_t public_snapshot;

typedef struct {
    char *bytes;
    size_t length;
} http_response_t;

static void ensure_lock(void)
{
    if (weather_lock == NULL) {
        weather_lock = xSemaphoreCreateMutex();
    }
}

static void publish(const weather_snapshot_t *snapshot)
{
    ensure_lock();
    xSemaphoreTake(weather_lock, portMAX_DELAY);
    public_snapshot = *snapshot;
    xSemaphoreGive(weather_lock);
    screen_request_refresh();
}

void weather_get_snapshot(weather_snapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return;
    }
    ensure_lock();
    xSemaphoreTake(weather_lock, portMAX_DELAY);
    *snapshot = public_snapshot;
    xSemaphoreGive(weather_lock);
}

static char *read_file(const char *path)
{
    FILE *file = fopen(path, "rb");
    if (file == NULL || fseek(file, 0, SEEK_END) != 0) {
        if (file != NULL) fclose(file);
        return NULL;
    }
    long length = ftell(file);
    if (length < 0 || fseek(file, 0, SEEK_SET) != 0) {
        fclose(file);
        return NULL;
    }
    char *text = heap_caps_malloc((size_t)length + 1,
                                  MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (text == NULL || fread(text, 1, (size_t)length, file) != (size_t)length) {
        free(text);
        fclose(file);
        return NULL;
    }
    text[length] = '\0';
    fclose(file);
    return text;
}

static char *load_api_key(void)
{
    char *text = read_file(WEATHER_CONFIG_PATH);
    if (text == NULL) {
        return NULL;
    }
    cJSON *root = cJSON_Parse(text);
    free(text);
    if (root == NULL) {
        return NULL;
    }
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(
        root, "openweathermap_api_key");
    char *key = NULL;
    if (cJSON_IsString(item) && item->valuestring[0] != '\0') {
        bool valid = true;
        for (const unsigned char *cursor =
                 (const unsigned char *)item->valuestring;
             *cursor != '\0'; cursor++) {
            if (!isalnum(*cursor) && *cursor != '_' && *cursor != '-') {
                valid = false;
                break;
            }
        }
        if (valid) {
            key = strdup(item->valuestring);
        }
    }
    cJSON_Delete(root);
    return key;
}

static bool parse_days(const cJSON *response, weather_snapshot_t *snapshot)
{
    const cJSON *offset = cJSON_GetObjectItemCaseSensitive(
        response, "timezone_offset");
    const cJSON *data = cJSON_GetObjectItemCaseSensitive(response, "data");
    if (!cJSON_IsNumber(offset) || !cJSON_IsArray(data)) {
        return false;
    }
    snapshot->timezone_offset_seconds = (int32_t)offset->valuedouble;
    snapshot->day_count = 0;
    cJSON *item;
    cJSON_ArrayForEach(item, data) {
        if (snapshot->day_count == WEATHER_DAY_COUNT) {
            break;
        }
        const cJSON *timestamp = cJSON_GetObjectItemCaseSensitive(item, "dt");
        const cJSON *temperature = cJSON_GetObjectItemCaseSensitive(item, "temp");
        const cJSON *day = cJSON_GetObjectItemCaseSensitive(temperature, "day");
        const cJSON *minimum = cJSON_GetObjectItemCaseSensitive(temperature, "min");
        const cJSON *maximum = cJSON_GetObjectItemCaseSensitive(temperature, "max");
        const cJSON *weather = cJSON_GetObjectItemCaseSensitive(item, "weather");
        const cJSON *condition = cJSON_GetArrayItem(weather, 0);
        const cJSON *condition_id = cJSON_GetObjectItemCaseSensitive(condition, "id");
        if (!cJSON_IsNumber(timestamp) || !cJSON_IsNumber(day) ||
            !cJSON_IsNumber(minimum) || !cJSON_IsNumber(maximum) ||
            !cJSON_IsNumber(condition_id)) {
            return false;
        }
        weather_day_t *output = &snapshot->days[snapshot->day_count++];
        output->timestamp = (int64_t)timestamp->valuedouble;
        output->temperature_tenths = (int16_t)lround(day->valuedouble * 10.0);
        output->minimum_tenths = (int16_t)lround(minimum->valuedouble * 10.0);
        output->maximum_tenths = (int16_t)lround(maximum->valuedouble * 10.0);
        output->condition_id = (uint16_t)condition_id->valuedouble;
    }
    return snapshot->day_count != 0;
}

static bool load_cache(weather_snapshot_t *snapshot)
{
    char *text = read_file(WEATHER_CACHE_PATH);
    if (text == NULL) {
        return false;
    }
    cJSON *root = cJSON_Parse(text);
    free(text);
    if (root == NULL) {
        return false;
    }
    const cJSON *version = cJSON_GetObjectItemCaseSensitive(root, "version");
    const cJSON *latitude = cJSON_GetObjectItemCaseSensitive(root, "latitude_e7");
    const cJSON *longitude = cJSON_GetObjectItemCaseSensitive(root, "longitude_e7");
    const cJSON *updated_at = cJSON_GetObjectItemCaseSensitive(root, "updated_at");
    const cJSON *response = cJSON_GetObjectItemCaseSensitive(root, "response");
    bool valid = cJSON_IsNumber(version) &&
        (version->valueint == 1 || version->valueint == 2) &&
        cJSON_IsNumber(latitude) && cJSON_IsNumber(longitude) &&
        cJSON_IsObject(response) && parse_days(response, snapshot);
    if (valid) {
        snapshot->cache_updated_at = cJSON_IsNumber(updated_at)
                                         ? (int64_t)updated_at->valuedouble
                                         : 0;
    }
    cJSON_Delete(root);
    return valid;
}

static esp_err_t save_cache(int32_t latitude_e7, int32_t longitude_e7,
                            int64_t updated_at, const cJSON *response)
{
    if (mkdir(WEATHER_CACHE_DIRECTORY, 0775) != 0 && errno != EEXIST) {
        return ESP_FAIL;
    }
    cJSON *root = cJSON_CreateObject();
    cJSON *response_copy = cJSON_Duplicate(response, true);
    if (root == NULL || response_copy == NULL ||
        cJSON_AddNumberToObject(root, "version", 2) == NULL ||
        cJSON_AddNumberToObject(root, "latitude_e7", latitude_e7) == NULL ||
        cJSON_AddNumberToObject(root, "longitude_e7", longitude_e7) == NULL ||
        cJSON_AddNumberToObject(root, "updated_at", (double)updated_at) == NULL) {
        cJSON_Delete(response_copy);
        cJSON_Delete(root);
        return ESP_ERR_NO_MEM;
    }
    cJSON_AddItemToObject(root, "response", response_copy);
    char *text = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (text == NULL) {
        return ESP_ERR_NO_MEM;
    }
    FILE *file = fopen(WEATHER_CACHE_TMP_PATH, "wb");
    bool written = file != NULL &&
        fwrite(text, 1, strlen(text), file) == strlen(text) &&
        fflush(file) == 0 && fsync(fileno(file)) == 0;
    if (file != NULL) fclose(file);
    cJSON_free(text);
    if (!written) {
        remove(WEATHER_CACHE_TMP_PATH);
        return ESP_FAIL;
    }
    remove(WEATHER_CACHE_BACKUP_PATH);
    bool had_cache = access(WEATHER_CACHE_PATH, F_OK) == 0;
    if (had_cache &&
        rename(WEATHER_CACHE_PATH, WEATHER_CACHE_BACKUP_PATH) != 0) {
        remove(WEATHER_CACHE_TMP_PATH);
        return ESP_FAIL;
    }
    if (rename(WEATHER_CACHE_TMP_PATH, WEATHER_CACHE_PATH) != 0) {
        if (had_cache) {
            rename(WEATHER_CACHE_BACKUP_PATH, WEATHER_CACHE_PATH);
        }
        remove(WEATHER_CACHE_TMP_PATH);
        return ESP_FAIL;
    }
    remove(WEATHER_CACHE_BACKUP_PATH);
    return ESP_OK;
}

static esp_err_t http_event(esp_http_client_event_t *event)
{
    http_response_t *response = event->user_data;
    if (event->event_id != HTTP_EVENT_ON_DATA || event->data_len == 0) {
        return ESP_OK;
    }
    char *resized = heap_caps_realloc(
        response->bytes, response->length + (size_t)event->data_len + 1,
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (resized == NULL) {
        return ESP_ERR_NO_MEM;
    }
    response->bytes = resized;
    memcpy(response->bytes + response->length, event->data,
           (size_t)event->data_len);
    response->length += (size_t)event->data_len;
    response->bytes[response->length] = '\0';
    return ESP_OK;
}

static cJSON *fetch_weather(const char *key, int32_t latitude_e7,
                            int32_t longitude_e7, int *http_status)
{
    if (http_status != NULL) {
        *http_status = 0;
    }
    size_t url_size = strlen(key) + 192;
    char *url = malloc(url_size);
    if (url == NULL) {
        return NULL;
    }
    snprintf(url, url_size,
             "https://api.openweathermap.org/data/4.0/onecall/timeline/1day"
             "?lat=%.7f&lon=%.7f&units=metric&lang=de&appid=%s",
             latitude_e7 / 10000000.0, longitude_e7 / 10000000.0, key);
    http_response_t response = {0};
    ESP_LOGI(TAG, "TLS heap before request: free=%u largest=%u",
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL |
                                                MALLOC_CAP_8BIT),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL |
                                                        MALLOC_CAP_8BIT));
    esp_http_client_config_t config = {
        .url = url,
        .event_handler = http_event,
        .user_data = &response,
        .crt_bundle_attach = esp_crt_bundle_attach,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == NULL) {
        free(url);
        return NULL;
    }
    esp_err_t result = esp_http_client_perform(client);
    int status = esp_http_client_get_status_code(client);
    if (http_status != NULL) {
        *http_status = status;
    }
    esp_http_client_cleanup(client);
    free(url);
    if (result != ESP_OK || status != 200 || response.bytes == NULL) {
        ESP_LOGW(TAG, "weather request failed: transport=%s status=%d",
                 esp_err_to_name(result), status);
        free(response.bytes);
        return NULL;
    }
    ESP_LOGI(TAG, "weather response received: %u bytes",
             (unsigned)response.length);
    cJSON *root = cJSON_Parse(response.bytes);
    free(response.bytes);
    return root;
}

static bool leap_year(int year)
{
    return year % 4 == 0 && (year % 100 != 0 || year % 400 == 0);
}

static void set_tls_clock_from_rtc(void)
{
    rtc_datetime_t rtc;
    if (ble_rtc_get_datetime(&rtc) != ESP_OK || rtc.year < 2000 ||
        rtc.year > 2099 || rtc.month < 1 || rtc.month > 12 ||
        rtc.day < 1 || rtc.day > 31) {
        return;
    }
    static const uint16_t days_before_month[] = {
        0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};
    int64_t days = 0;
    for (int year = 1970; year < rtc.year; year++) {
        days += leap_year(year) ? 366 : 365;
    }
    days += days_before_month[rtc.month - 1] + rtc.day - 1;
    if (rtc.month > 2 && leap_year(rtc.year)) {
        days++;
    }
    struct timeval now = {
        .tv_sec = days * 86400 + rtc.hour * 3600 + rtc.minute * 60 +
                  rtc.second,
    };
    settimeofday(&now, NULL);
}

static void weather_task(void *parameter)
{
    (void)parameter;
    weather_snapshot_t snapshot = {.state = WEATHER_LOADING};
    if (!gps_get_best_position(&snapshot.latitude_e7, &snapshot.longitude_e7,
                               &snapshot.position_is_live)) {
        snapshot.state = WEATHER_NO_POSITION;
        publish(&snapshot);
        weather_task_handle = NULL;
        vTaskDelete(NULL);
        return;
    }

    set_tls_clock_from_rtc();
    int64_t now = (int64_t)time(NULL);

    esp_err_t sd_result = sd_storage_acquire();
    if (sd_result != ESP_OK) {
        snapshot.state = WEATHER_SD_ERROR;
        publish(&snapshot);
        weather_task_handle = NULL;
        vTaskDelete(NULL);
        return;
    }
    char *key = load_api_key();
    bool cache_valid = load_cache(&snapshot);
    sd_storage_release();

    if (cache_valid) {
        snapshot.state = WEATHER_READY_CACHE;
        publish(&snapshot);
        bool cache_fresh = snapshot.cache_updated_at > 0 &&
                           now >= snapshot.cache_updated_at &&
                           now - snapshot.cache_updated_at <
                               WEATHER_CACHE_REFRESH_SECONDS;
        if (cache_fresh) {
            ESP_LOGI(TAG, "using fresh weather cache; network refresh deferred");
            free(key);
            weather_task_handle = NULL;
            vTaskDelete(NULL);
            return;
        }
    }

    watch_wifi_status_t wifi;
    watch_wifi_get_status(&wifi);
    while (wifi.requested &&
           (wifi.state == WATCH_WIFI_OFF ||
            wifi.state == WATCH_WIFI_LOADING ||
            wifi.state == WATCH_WIFI_CONNECTING)) {
        watch_wifi_wait_for_status_change();
        watch_wifi_get_status(&wifi);
    }
    if (wifi.state != WATCH_WIFI_CONNECTED) {
        snapshot.state = cache_valid ? WEATHER_READY_CACHE : WEATHER_NO_CACHE;
        publish(&snapshot);
        free(key);
        weather_task_handle = NULL;
        vTaskDelete(NULL);
        return;
    }
    if (key == NULL) {
        snapshot.state = cache_valid ? WEATHER_READY_CACHE : WEATHER_NO_API_KEY;
        publish(&snapshot);
        weather_task_handle = NULL;
        vTaskDelete(NULL);
        return;
    }

    esp_err_t transfer_result = watch_wifi_set_transfer_active(true);
    if (transfer_result != ESP_OK) {
        memset(key, 0, strlen(key));
        free(key);
        snapshot.state = cache_valid ? WEATHER_READY_CACHE
                                     : WEATHER_NETWORK_ERROR;
        publish(&snapshot);
        weather_task_handle = NULL;
        vTaskDelete(NULL);
        return;
    }

    int http_status = 0;
    cJSON *response = fetch_weather(key, snapshot.latitude_e7,
                                    snapshot.longitude_e7, &http_status);
    memset(key, 0, strlen(key));
    free(key);
    weather_snapshot_t online = snapshot;
    if (response != NULL && parse_days(response, &online)) {
        online.cache_updated_at = (int64_t)time(NULL);
        online.state = WEATHER_READY_ONLINE;
        if (sd_storage_acquire() == ESP_OK) {
            esp_err_t save_result = save_cache(
                online.latitude_e7, online.longitude_e7,
                online.cache_updated_at, response);
            sd_storage_release();
            if (save_result != ESP_OK) {
                ESP_LOGW(TAG, "weather cache write failed: %s",
                         esp_err_to_name(save_result));
            }
        }
        cJSON_Delete(response);
        watch_wifi_set_transfer_active(false);
        ESP_LOGI(TAG,
                 "heap before weather display refresh: free=%u largest=%u",
                 (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                 (unsigned)heap_caps_get_largest_free_block(
                     MALLOC_CAP_INTERNAL));
        publish(&online);
    } else {
        cJSON_Delete(response);
        watch_wifi_set_transfer_active(false);
        snapshot.state = cache_valid
                             ? WEATHER_READY_CACHE
                             : ((http_status == 401 || http_status == 403)
                                    ? WEATHER_API_ACCESS_ERROR
                                    : WEATHER_NETWORK_ERROR);
        publish(&snapshot);
    }
    weather_task_handle = NULL;
    vTaskDelete(NULL);
}

esp_err_t weather_start(void)
{
    ensure_lock();
    if (weather_lock == NULL) {
        return ESP_ERR_NO_MEM;
    }
    if (weather_task_handle != NULL) {
        return ESP_OK;
    }
    weather_snapshot_t loading;
    weather_get_snapshot(&loading);
    if (loading.state != WEATHER_READY_ONLINE &&
        loading.state != WEATHER_READY_CACHE) {
        memset(&loading, 0, sizeof(loading));
        loading.state = WEATHER_LOADING;
        publish(&loading);
    }
    if (xTaskCreate(weather_task, "weather", 6144, NULL, 4,
                    &weather_task_handle) != pdPASS) {
        weather_task_handle = NULL;
        loading.state = WEATHER_DATA_ERROR;
        publish(&loading);
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}
