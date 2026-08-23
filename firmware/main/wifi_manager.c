#include "wifi_manager.h"

#include <ctype.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#include "cJSON.h"
#include "esp_event.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "screen_control.h"
#include "sd_storage.h"

#define WIFI_CONFIG_PATH "/sdcard/ultrawatch/config.txt"
#define WIFI_CONFIG_TMP_PATH "/sdcard/ultrawatch/config.tmp"
#define WIFI_CONFIG_BACKUP_PATH "/sdcard/ultrawatch/config.bak"

#define WIFI_EVENT_CONNECTED (1U << 0)
#define WIFI_EVENT_DISCONNECTED (1U << 1)
#define WIFI_COMMAND_RETRY (1U << 2)
#define WIFI_COMMAND_STOP (1U << 3)

static const char *TAG = "watch_wifi";
static SemaphoreHandle_t state_lock;
static SemaphoreHandle_t profile_lock;
static EventGroupHandle_t wifi_events;
static TaskHandle_t wifi_task_handle;
static watch_wifi_status_t public_status = {
    .state = WATCH_WIFI_OFF,
    .rssi = 127,
    .active_profile = UINT32_MAX,
};
static watch_wifi_status_callback_t status_callback;
static esp_netif_t *station_netif;
static esp_event_handler_instance_t wifi_handler;
static esp_event_handler_instance_t ip_handler;
static bool driver_initialized;

static void ensure_locks(void)
{
    if (state_lock == NULL) state_lock = xSemaphoreCreateMutex();
    if (profile_lock == NULL) profile_lock = xSemaphoreCreateMutex();
    if (wifi_events == NULL) wifi_events = xEventGroupCreate();
}

static void publish_status(watch_wifi_state_t state,
                           watch_wifi_error_t error, uint32_t active_profile)
{
    ensure_locks();
    xSemaphoreTake(state_lock, portMAX_DELAY);
    public_status.state = state;
    public_status.error = error;
    public_status.active_profile = active_profile;
    if (state != WATCH_WIFI_CONNECTED) {
        public_status.rssi = 127;
        memset(public_status.ipv4, 0, sizeof(public_status.ipv4));
    }
    watch_wifi_status_callback_t callback = status_callback;
    xSemaphoreGive(state_lock);
    screen_request_refresh();
    if (callback != NULL) callback();
}

void watch_wifi_get_status(watch_wifi_status_t *status)
{
    if (status == NULL) return;
    ensure_locks();
    xSemaphoreTake(state_lock, portMAX_DELAY);
    *status = public_status;
    xSemaphoreGive(state_lock);
    if (status->state == WATCH_WIFI_CONNECTED) {
        wifi_ap_record_t access_point;
        if (esp_wifi_sta_get_ap_info(&access_point) == ESP_OK) {
            status->rssi = access_point.rssi;
            xSemaphoreTake(state_lock, portMAX_DELAY);
            public_status.rssi = access_point.rssi;
            xSemaphoreGive(state_lock);
        }
    }
}

void watch_wifi_set_status_callback(watch_wifi_status_callback_t callback)
{
    ensure_locks();
    xSemaphoreTake(state_lock, portMAX_DELAY);
    status_callback = callback;
    xSemaphoreGive(state_lock);
}

static bool valid_password(const char *password)
{
    size_t length = strlen(password);
    if (length == 0 || (length >= 8 && length <= 63)) return true;
    if (length != 64) return false;
    for (size_t index = 0; index < length; index++) {
        if (!isxdigit((unsigned char)password[index])) return false;
    }
    return true;
}

static bool valid_utf8(const char *text)
{
    const unsigned char *bytes = (const unsigned char *)text;
    size_t length = strlen(text);
    size_t index = 0;
    while (index < length) {
        size_t remaining = length - index;
        const unsigned char *current = bytes + index;
        if (*current <= 0x7f) {
            index++;
            continue;
        }
        if (remaining >= 2 && *current >= 0xc2 && *current <= 0xdf &&
            current[1] >= 0x80 && current[1] <= 0xbf) {
            index += 2;
            continue;
        }
        if (remaining >= 3 && *current == 0xe0 &&
            current[1] >= 0xa0 && current[1] <= 0xbf &&
            current[2] >= 0x80 && current[2] <= 0xbf) {
            index += 3;
            continue;
        }
        if (remaining >= 3 &&
            ((*current >= 0xe1 && *current <= 0xec) ||
             (*current >= 0xee && *current <= 0xef)) &&
            current[1] >= 0x80 && current[1] <= 0xbf &&
            current[2] >= 0x80 && current[2] <= 0xbf) {
            index += 3;
            continue;
        }
        if (remaining >= 3 && *current == 0xed &&
            current[1] >= 0x80 && current[1] <= 0x9f &&
            current[2] >= 0x80 && current[2] <= 0xbf) {
            index += 3;
            continue;
        }
        if (remaining >= 4 && *current == 0xf0 &&
            current[1] >= 0x90 && current[1] <= 0xbf &&
            current[2] >= 0x80 && current[2] <= 0xbf &&
            current[3] >= 0x80 && current[3] <= 0xbf) {
            index += 4;
            continue;
        }
        if (remaining >= 4 && *current >= 0xf1 && *current <= 0xf3 &&
            current[1] >= 0x80 && current[1] <= 0xbf &&
            current[2] >= 0x80 && current[2] <= 0xbf &&
            current[3] >= 0x80 && current[3] <= 0xbf) {
            index += 4;
            continue;
        }
        if (remaining >= 4 && *current == 0xf4 &&
            current[1] >= 0x80 && current[1] <= 0x8f &&
            current[2] >= 0x80 && current[2] <= 0xbf &&
            current[3] >= 0x80 && current[3] <= 0xbf) {
            index += 4;
            continue;
        }
        return false;
    }
    return true;
}

static bool valid_profile(const watch_wifi_profile_t *profile)
{
    if (profile == NULL) return false;
    size_t ssid_length = strnlen(profile->ssid, sizeof(profile->ssid));
    size_t password_length = strnlen(profile->password,
                                     sizeof(profile->password));
    return ssid_length >= 1 && ssid_length <= 32 &&
           password_length < sizeof(profile->password) &&
           valid_utf8(profile->ssid) && valid_utf8(profile->password) &&
           valid_password(profile->password);
}

static char *read_text_file(const char *path)
{
    FILE *file = fopen(path, "rb");
    if (file == NULL) return NULL;
    if (fseek(file, 0, SEEK_END) != 0) {
        fclose(file);
        return NULL;
    }
    long size = ftell(file);
    if (size < 0 || fseek(file, 0, SEEK_SET) != 0) {
        fclose(file);
        return NULL;
    }
    char *text = malloc((size_t)size + 1);
    if (text == NULL) {
        fclose(file);
        return NULL;
    }
    if (fread(text, 1, (size_t)size, file) != (size_t)size) {
        free(text);
        fclose(file);
        return NULL;
    }
    text[size] = '\0';
    fclose(file);
    return text;
}

static bool json_to_profile(const cJSON *item, watch_wifi_profile_t *profile)
{
    const cJSON *ssid = cJSON_GetObjectItemCaseSensitive(item, "ssid");
    const cJSON *password = cJSON_GetObjectItemCaseSensitive(item, "password");
    if (!cJSON_IsString(ssid) || !cJSON_IsString(password) ||
        ssid->valuestring == NULL || password->valuestring == NULL) {
        return false;
    }
    if (strlen(ssid->valuestring) > 32 || strlen(password->valuestring) > 64) {
        return false;
    }
    memset(profile, 0, sizeof(*profile));
    memcpy(profile->ssid, ssid->valuestring, strlen(ssid->valuestring));
    memcpy(profile->password, password->valuestring,
           strlen(password->valuestring));
    return valid_profile(profile);
}

static esp_err_t validate_root(cJSON *root)
{
    const cJSON *version = cJSON_GetObjectItemCaseSensitive(root, "version");
    cJSON *networks = cJSON_GetObjectItemCaseSensitive(root, "networks");
    if (!cJSON_IsNumber(version) || version->valuedouble != 1 ||
        !cJSON_IsArray(networks)) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    cJSON *item;
    cJSON_ArrayForEach(item, networks) {
        watch_wifi_profile_t profile;
        if (!cJSON_IsObject(item) || !json_to_profile(item, &profile)) {
            return ESP_ERR_INVALID_RESPONSE;
        }
        cJSON *other = item->next;
        while (other != NULL) {
            watch_wifi_profile_t candidate;
            if (json_to_profile(other, &candidate) &&
                strcmp(profile.ssid, candidate.ssid) == 0) {
                return ESP_ERR_INVALID_RESPONSE;
            }
            other = other->next;
        }
    }
    return ESP_OK;
}

static cJSON *parse_config_path(const char *path)
{
    char *text = read_text_file(path);
    if (text == NULL) return NULL;
    cJSON *root = cJSON_Parse(text);
    free(text);
    if (root == NULL || validate_root(root) != ESP_OK) {
        cJSON_Delete(root);
        return NULL;
    }
    return root;
}

static cJSON *load_config(bool allow_empty, esp_err_t *error)
{
    cJSON *root = parse_config_path(WIFI_CONFIG_PATH);
    if (root != NULL) {
        *error = ESP_OK;
        return root;
    }
    root = parse_config_path(WIFI_CONFIG_BACKUP_PATH);
    if (root != NULL) {
        remove(WIFI_CONFIG_PATH);
        if (rename(WIFI_CONFIG_BACKUP_PATH, WIFI_CONFIG_PATH) != 0) {
            ESP_LOGW(TAG, "cannot restore Wi-Fi backup: errno=%d", errno);
        }
        *error = ESP_OK;
        return root;
    }
    if (allow_empty && access(WIFI_CONFIG_PATH, F_OK) != 0 &&
        access(WIFI_CONFIG_BACKUP_PATH, F_OK) != 0) {
        root = cJSON_CreateObject();
        if (root == NULL || cJSON_AddNumberToObject(root, "version", 1) == NULL ||
            cJSON_AddArrayToObject(root, "networks") == NULL) {
            cJSON_Delete(root);
            root = NULL;
            *error = ESP_ERR_NO_MEM;
        } else {
            *error = ESP_OK;
        }
        return root;
    }
    *error = access(WIFI_CONFIG_PATH, F_OK) == 0
                 ? ESP_ERR_INVALID_RESPONSE : ESP_ERR_NOT_FOUND;
    return NULL;
}

static esp_err_t save_config(cJSON *root)
{
    ESP_RETURN_ON_ERROR(validate_root(root), TAG, "Wi-Fi config invalid");
    char *text = cJSON_Print(root);
    if (text == NULL) return ESP_ERR_NO_MEM;
    if (mkdir("/sdcard/ultrawatch", 0775) != 0 && errno != EEXIST) {
        cJSON_free(text);
        return ESP_FAIL;
    }
    FILE *file = fopen(WIFI_CONFIG_TMP_PATH, "wb");
    if (file == NULL) {
        cJSON_free(text);
        return ESP_FAIL;
    }
    size_t length = strlen(text);
    bool written = fwrite(text, 1, length, file) == length &&
                   fflush(file) == 0 && fsync(fileno(file)) == 0;
    cJSON_free(text);
    if (fclose(file) != 0) written = false;
    if (!written) {
        remove(WIFI_CONFIG_TMP_PATH);
        return ESP_FAIL;
    }

    remove(WIFI_CONFIG_BACKUP_PATH);
    bool had_main = access(WIFI_CONFIG_PATH, F_OK) == 0;
    if (had_main && rename(WIFI_CONFIG_PATH, WIFI_CONFIG_BACKUP_PATH) != 0) {
        remove(WIFI_CONFIG_TMP_PATH);
        return ESP_FAIL;
    }
    if (rename(WIFI_CONFIG_TMP_PATH, WIFI_CONFIG_PATH) != 0) {
        if (had_main) rename(WIFI_CONFIG_BACKUP_PATH, WIFI_CONFIG_PATH);
        remove(WIFI_CONFIG_TMP_PATH);
        return ESP_FAIL;
    }
    remove(WIFI_CONFIG_BACKUP_PATH);
    return ESP_OK;
}

static esp_err_t with_config(cJSON **root, bool allow_empty)
{
    esp_err_t result = sd_storage_acquire();
    if (result != ESP_OK) return result;
    *root = load_config(allow_empty, &result);
    if (*root == NULL) sd_storage_release();
    return result;
}

static void configuration_changed(void)
{
    ensure_locks();
    xSemaphoreTake(state_lock, portMAX_DELAY);
    bool active = public_status.requested;
    xSemaphoreGive(state_lock);
    if (active) xEventGroupSetBits(wifi_events, WIFI_COMMAND_RETRY);
}

esp_err_t watch_wifi_profile_count(uint32_t *count)
{
    if (count == NULL) return ESP_ERR_INVALID_ARG;
    ensure_locks();
    xSemaphoreTake(profile_lock, portMAX_DELAY);
    cJSON *root;
    esp_err_t result = with_config(&root, true);
    if (result == ESP_OK) {
        *count = (uint32_t)cJSON_GetArraySize(
            cJSON_GetObjectItemCaseSensitive(root, "networks"));
        cJSON_Delete(root);
        sd_storage_release();
    }
    xSemaphoreGive(profile_lock);
    return result;
}

esp_err_t watch_wifi_profile_get(uint32_t index, watch_wifi_profile_t *profile)
{
    if (profile == NULL) return ESP_ERR_INVALID_ARG;
    ensure_locks();
    xSemaphoreTake(profile_lock, portMAX_DELAY);
    cJSON *root;
    esp_err_t result = with_config(&root, false);
    if (result == ESP_OK) {
        cJSON *item = cJSON_GetArrayItem(
            cJSON_GetObjectItemCaseSensitive(root, "networks"), (int)index);
        result = item != NULL && json_to_profile(item, profile)
                     ? ESP_OK : ESP_ERR_NOT_FOUND;
        cJSON_Delete(root);
        sd_storage_release();
    }
    xSemaphoreGive(profile_lock);
    return result;
}

esp_err_t watch_wifi_profile_put(uint32_t index, bool append,
                                 const watch_wifi_profile_t *profile)
{
    if (!valid_profile(profile)) return ESP_ERR_INVALID_ARG;
    ensure_locks();
    xSemaphoreTake(profile_lock, portMAX_DELAY);
    cJSON *root;
    esp_err_t result = with_config(&root, true);
    if (result == ESP_OK) {
        cJSON *networks = cJSON_GetObjectItemCaseSensitive(root, "networks");
        cJSON *replacement = cJSON_CreateObject();
        if (replacement == NULL ||
            cJSON_AddStringToObject(replacement, "ssid", profile->ssid) == NULL ||
            cJSON_AddStringToObject(replacement, "password",
                                    profile->password) == NULL) {
            cJSON_Delete(replacement);
            result = ESP_ERR_NO_MEM;
        } else if (append) {
            cJSON_AddItemToArray(networks, replacement);
        } else if (index >= (uint32_t)cJSON_GetArraySize(networks)) {
            cJSON_Delete(replacement);
            result = ESP_ERR_NOT_FOUND;
        } else {
            cJSON_ReplaceItemInArray(networks, (int)index, replacement);
        }
        if (result == ESP_OK) result = save_config(root);
        cJSON_Delete(root);
        sd_storage_release();
    }
    xSemaphoreGive(profile_lock);
    if (result == ESP_OK) configuration_changed();
    return result;
}

esp_err_t watch_wifi_profile_delete(uint32_t index)
{
    ensure_locks();
    xSemaphoreTake(profile_lock, portMAX_DELAY);
    cJSON *root;
    esp_err_t result = with_config(&root, false);
    if (result == ESP_OK) {
        cJSON *networks = cJSON_GetObjectItemCaseSensitive(root, "networks");
        if (index >= (uint32_t)cJSON_GetArraySize(networks)) {
            result = ESP_ERR_NOT_FOUND;
        } else {
            cJSON_DeleteItemFromArray(networks, (int)index);
            result = save_config(root);
        }
        cJSON_Delete(root);
        sd_storage_release();
    }
    xSemaphoreGive(profile_lock);
    if (result == ESP_OK) configuration_changed();
    return result;
}

esp_err_t watch_wifi_profile_move(uint32_t index, uint32_t destination)
{
    ensure_locks();
    xSemaphoreTake(profile_lock, portMAX_DELAY);
    cJSON *root;
    esp_err_t result = with_config(&root, false);
    if (result == ESP_OK) {
        cJSON *networks = cJSON_GetObjectItemCaseSensitive(root, "networks");
        uint32_t count = (uint32_t)cJSON_GetArraySize(networks);
        if (index >= count || destination >= count) {
            result = ESP_ERR_NOT_FOUND;
        } else if (index != destination) {
            cJSON *item = cJSON_DetachItemFromArray(networks, (int)index);
            cJSON_InsertItemInArray(networks, (int)destination, item);
            result = save_config(root);
        }
        cJSON_Delete(root);
        sd_storage_release();
    }
    xSemaphoreGive(profile_lock);
    if (result == ESP_OK) configuration_changed();
    return result;
}

static esp_err_t load_profiles(watch_wifi_profile_t **profiles,
                               uint32_t *count)
{
    ensure_locks();
    xSemaphoreTake(profile_lock, portMAX_DELAY);
    cJSON *root;
    esp_err_t result = with_config(&root, false);
    if (result == ESP_OK) {
        cJSON *networks = cJSON_GetObjectItemCaseSensitive(root, "networks");
        *count = (uint32_t)cJSON_GetArraySize(networks);
        if (*count == 0) {
            result = ESP_ERR_INVALID_SIZE;
        } else {
            *profiles = calloc(*count, sizeof(**profiles));
            if (*profiles == NULL) {
                result = ESP_ERR_NO_MEM;
            } else {
                for (uint32_t index = 0; index < *count; index++) {
                    if (!json_to_profile(cJSON_GetArrayItem(networks, (int)index),
                                         &(*profiles)[index])) {
                        result = ESP_ERR_INVALID_RESPONSE;
                        break;
                    }
                }
            }
        }
        cJSON_Delete(root);
        sd_storage_release();
    }
    xSemaphoreGive(profile_lock);
    if (result != ESP_OK) {
        free(*profiles);
        *profiles = NULL;
    }
    return result;
}

static void wifi_event(void *argument, esp_event_base_t base,
                       int32_t id, void *data)
{
    (void)argument;
    (void)data;
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        xEventGroupSetBits(wifi_events, WIFI_EVENT_DISCONNECTED);
    }
}

static void ip_event(void *argument, esp_event_base_t base,
                     int32_t id, void *data)
{
    (void)argument;
    if (base != IP_EVENT || id != IP_EVENT_STA_GOT_IP) return;
    ip_event_got_ip_t *event = data;
    uint32_t address = event->ip_info.ip.addr;
    xSemaphoreTake(state_lock, portMAX_DELAY);
    public_status.ipv4[0] = address & 0xff;
    public_status.ipv4[1] = (address >> 8) & 0xff;
    public_status.ipv4[2] = (address >> 16) & 0xff;
    public_status.ipv4[3] = (address >> 24) & 0xff;
    xSemaphoreGive(state_lock);
    xEventGroupSetBits(wifi_events, WIFI_EVENT_CONNECTED);
}

static esp_err_t start_driver(void)
{
    if (driver_initialized) return ESP_OK;
    bool wifi_initialized = false;
    bool wifi_started = false;
    bool wifi_handler_registered = false;
    bool ip_handler_registered = false;
    esp_err_t result = esp_netif_init();
    if (result != ESP_OK && result != ESP_ERR_INVALID_STATE) return result;
    result = esp_event_loop_create_default();
    if (result != ESP_OK && result != ESP_ERR_INVALID_STATE) return result;
    station_netif = esp_netif_create_default_wifi_sta();
    if (station_netif == NULL) return ESP_ERR_NO_MEM;
    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    result = esp_wifi_init(&init);
    if (result != ESP_OK) goto fail;
    wifi_initialized = true;
    result = esp_wifi_set_storage(WIFI_STORAGE_RAM);
    if (result != ESP_OK) goto fail;
    result = esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event, NULL, &wifi_handler);
    if (result != ESP_OK) goto fail;
    wifi_handler_registered = true;
    result = esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, ip_event, NULL, &ip_handler);
    if (result != ESP_OK) goto fail;
    ip_handler_registered = true;
    result = esp_wifi_set_mode(WIFI_MODE_STA);
    if (result != ESP_OK) goto fail;
    result = esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
    if (result != ESP_OK) goto fail;
    result = esp_wifi_start();
    if (result != ESP_OK) goto fail;
    wifi_started = true;
    driver_initialized = true;
    return ESP_OK;

fail:
    ESP_LOGE(TAG, "Wi-Fi driver start failed: %s", esp_err_to_name(result));
    if (wifi_started) esp_wifi_stop();
    if (ip_handler_registered) {
        esp_event_handler_instance_unregister(IP_EVENT,
                                              IP_EVENT_STA_GOT_IP, ip_handler);
    }
    if (wifi_handler_registered) {
        esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                              wifi_handler);
    }
    if (wifi_initialized) esp_wifi_deinit();
    if (station_netif != NULL) esp_netif_destroy_default_wifi(station_netif);
    station_netif = NULL;
    return result;
}

static void stop_driver(void)
{
    if (!driver_initialized) return;
    esp_wifi_disconnect();
    esp_wifi_stop();
    esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                          wifi_handler);
    esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                          ip_handler);
    esp_wifi_deinit();
    if (station_netif != NULL) esp_netif_destroy_default_wifi(station_netif);
    station_netif = NULL;
    driver_initialized = false;
}

static bool requested(void)
{
    xSemaphoreTake(state_lock, portMAX_DELAY);
    bool value = public_status.requested;
    xSemaphoreGive(state_lock);
    return value;
}

static bool run_connection_pass(void)
{
    publish_status(WATCH_WIFI_LOADING, WATCH_WIFI_ERROR_NONE, UINT32_MAX);
    watch_wifi_profile_t *profiles = NULL;
    uint32_t count = 0;
    esp_err_t result = load_profiles(&profiles, &count);
    if (result != ESP_OK) {
        watch_wifi_state_t state = WATCH_WIFI_SD_ERROR;
        watch_wifi_error_t error = WATCH_WIFI_ERROR_NO_CARD;
        if (result == ESP_ERR_NOT_FOUND) {
            state = WATCH_WIFI_CONFIG_ERROR;
            error = WATCH_WIFI_ERROR_FILE_MISSING;
        } else if (result == ESP_ERR_INVALID_SIZE) {
            state = WATCH_WIFI_CONFIG_ERROR;
            error = WATCH_WIFI_ERROR_NO_PROFILES;
        } else if (result == ESP_ERR_INVALID_RESPONSE) {
            state = WATCH_WIFI_CONFIG_ERROR;
            error = WATCH_WIFI_ERROR_INVALID_CONFIG;
        }
        publish_status(state, error, UINT32_MAX);
        return false;
    }
    result = start_driver();
    if (result != ESP_OK) {
        free(profiles);
        publish_status(WATCH_WIFI_DRIVER_ERROR, WATCH_WIFI_ERROR_DRIVER,
                       UINT32_MAX);
        return false;
    }

    bool connected = false;
    for (uint32_t index = 0; index < count && requested(); index++) {
        wifi_config_t config = {0};
        size_t ssid_length = strlen(profiles[index].ssid);
        size_t password_length = strlen(profiles[index].password);
        memcpy(config.sta.ssid, profiles[index].ssid, ssid_length);
        memcpy(config.sta.password, profiles[index].password, password_length);
        config.sta.scan_method = WIFI_ALL_CHANNEL_SCAN;
        config.sta.sort_method = WIFI_CONNECT_AP_BY_SECURITY;
        xEventGroupClearBits(wifi_events, WIFI_EVENT_CONNECTED |
                                          WIFI_EVENT_DISCONNECTED);
        publish_status(WATCH_WIFI_CONNECTING, WATCH_WIFI_ERROR_NONE, index);
        if (esp_wifi_set_config(WIFI_IF_STA, &config) != ESP_OK ||
            esp_wifi_connect() != ESP_OK) {
            continue;
        }
        EventBits_t bits = xEventGroupWaitBits(
            wifi_events, WIFI_EVENT_CONNECTED | WIFI_EVENT_DISCONNECTED |
                             WIFI_COMMAND_STOP | WIFI_COMMAND_RETRY,
            pdTRUE, pdFALSE, portMAX_DELAY);
        if (bits & WIFI_COMMAND_STOP) {
            xEventGroupSetBits(wifi_events, WIFI_COMMAND_STOP);
            break;
        }
        if (bits & WIFI_COMMAND_RETRY) {
            xEventGroupSetBits(wifi_events, WIFI_COMMAND_RETRY);
            break;
        }
        if (bits & WIFI_EVENT_CONNECTED) {
            publish_status(WATCH_WIFI_CONNECTED, WATCH_WIFI_ERROR_NONE, index);
            connected = true;
            break;
        }
    }
    for (uint32_t index = 0; index < count; index++) {
        memset(&profiles[index], 0, sizeof(profiles[index]));
    }
    free(profiles);
    if (!connected && requested()) {
        publish_status(WATCH_WIFI_DISCONNECTED,
                       WATCH_WIFI_ERROR_CONNECTIONS_FAILED, UINT32_MAX);
    }
    return connected;
}

static void wifi_task(void *argument)
{
    (void)argument;
    while (requested()) {
        bool connected = run_connection_pass();
        EventBits_t bits = xEventGroupWaitBits(
            wifi_events, WIFI_EVENT_DISCONNECTED | WIFI_COMMAND_RETRY |
                             WIFI_COMMAND_STOP,
            pdTRUE, pdFALSE, portMAX_DELAY);
        if (!requested() || (bits & WIFI_COMMAND_STOP)) break;
        if (connected && (bits & WIFI_EVENT_DISCONNECTED)) {
            ESP_LOGI(TAG, "Wi-Fi disconnected; starting one recovery pass");
        }
        stop_driver();
    }
    stop_driver();
    publish_status(WATCH_WIFI_OFF, WATCH_WIFI_ERROR_NONE, UINT32_MAX);
    wifi_task_handle = NULL;
    vTaskDelete(NULL);
}

esp_err_t watch_wifi_set_enabled(bool enabled)
{
    ensure_locks();
    if (state_lock == NULL || profile_lock == NULL || wifi_events == NULL) {
        return ESP_ERR_NO_MEM;
    }
    xSemaphoreTake(state_lock, portMAX_DELAY);
    public_status.requested = enabled;
    xSemaphoreGive(state_lock);
    if (!enabled) {
        xEventGroupSetBits(wifi_events, WIFI_COMMAND_STOP);
        if (wifi_task_handle == NULL) {
            publish_status(WATCH_WIFI_OFF, WATCH_WIFI_ERROR_NONE, UINT32_MAX);
        }
        return ESP_OK;
    }
    if (wifi_task_handle != NULL) {
        xEventGroupSetBits(wifi_events, WIFI_COMMAND_RETRY);
        return ESP_OK;
    }
    xEventGroupClearBits(wifi_events, WIFI_COMMAND_STOP | WIFI_COMMAND_RETRY |
                                      WIFI_EVENT_CONNECTED |
                                      WIFI_EVENT_DISCONNECTED);
    if (xTaskCreate(wifi_task, "watch_wifi", 6144, NULL, 3,
                    &wifi_task_handle) != pdPASS) {
        xSemaphoreTake(state_lock, portMAX_DELAY);
        public_status.requested = false;
        xSemaphoreGive(state_lock);
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}
