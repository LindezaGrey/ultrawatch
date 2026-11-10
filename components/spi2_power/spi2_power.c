/*
 * spi2_power.c - see spi2_power.h for the whole-file rationale.
 */
#include "spi2_power.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "spi2_power";

#define SPI2_POWER_MAX_RAILS 4

typedef struct {
    axp2101_rail_t       rail;
    spi2_power_policy_t  policy;
    spi2_power_need_fn_t need_fn;
    bool                 in_use;
} spi2_power_entry_t;

static i2c_master_dev_handle_t s_pmu;
static SemaphoreHandle_t       s_mux;
static int                     s_holders;
static spi2_power_entry_t      s_rails[SPI2_POWER_MAX_RAILS];

esp_err_t spi2_power_init(i2c_master_dev_handle_t pmu)
{
    s_pmu = pmu;
    s_holders = 0;
    memset(s_rails, 0, sizeof(s_rails));
    if (!s_mux) {
        s_mux = xSemaphoreCreateMutex();
        if (!s_mux) {
            return ESP_ERR_NO_MEM;
        }
    }
    return ESP_OK;
}

static spi2_power_entry_t *find_entry(axp2101_rail_t rail)
{
    for (int i = 0; i < SPI2_POWER_MAX_RAILS; i++) {
        if (s_rails[i].in_use && s_rails[i].rail == rail) {
            return &s_rails[i];
        }
    }
    return NULL;
}

esp_err_t spi2_power_register(axp2101_rail_t rail, spi2_power_policy_t policy,
                               spi2_power_need_fn_t need_fn)
{
    if (rail >= AXP2101_RAIL_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    if (find_entry(rail) != NULL) {
        ESP_LOGE(TAG, "rail %d already registered", (int)rail);
        return ESP_ERR_INVALID_STATE;
    }
    for (int i = 0; i < SPI2_POWER_MAX_RAILS; i++) {
        if (!s_rails[i].in_use) {
            s_rails[i] = (spi2_power_entry_t){
                .rail = rail, .policy = policy, .need_fn = need_fn, .in_use = true,
            };
            return ESP_OK;
        }
    }
    ESP_LOGE(TAG, "no room to register rail %d (SPI2_POWER_MAX_RAILS=%d)",
             (int)rail, SPI2_POWER_MAX_RAILS);
    return ESP_ERR_NO_MEM;
}

/* Enables `rail` if it isn't already on, and reports back whether it just
 * transitioned off->on (so the caller knows whether a settle delay is
 * needed) - a rail already on is left untouched, no delay owed. */
static bool enable_and_report_transition(axp2101_rail_t rail)
{
    bool was_on = false;
    axp2101_is_rail_enabled(s_pmu, rail, &was_on);
    if (!was_on) {
        axp2101_enable_rail(s_pmu, rail, true);
    }
    return !was_on;
}

/* Raises every SHARED rail whose need_fn() (or absence of one) says yes,
 * plus `owned_rail` if it names a registered OWNED rail. Returns true if
 * any rail actually transitioned off->on (so the caller knows whether to
 * pay a settle delay). Called with s_mux held. */
static bool raise_rails(axp2101_rail_t owned_rail)
{
    bool any_turned_on = false;
    for (int i = 0; i < SPI2_POWER_MAX_RAILS; i++) {
        spi2_power_entry_t *e = &s_rails[i];
        if (!e->in_use || e->policy != SPI2_POWER_SHARED) {
            continue;
        }
        bool needed = e->need_fn ? e->need_fn() : true;
        if (needed && enable_and_report_transition(e->rail)) {
            any_turned_on = true;
        }
    }
    if (owned_rail < AXP2101_RAIL_MAX) {
        spi2_power_entry_t *e = find_entry(owned_rail);
        if (e && e->policy == SPI2_POWER_OWNED) {
            if (enable_and_report_transition(e->rail)) {
                any_turned_on = true;
            }
        } else {
            ESP_LOGW(TAG, "hold: rail %d is not a registered OWNED rail", (int)owned_rail);
        }
    }
    return any_turned_on;
}

/* Re-evaluates every SHARED rail's need_fn() now and lowers whichever say
 * no. OWNED rails are never touched here - see spi2_power.h. Called with
 * s_mux held. */
static void lower_rails(void)
{
    for (int i = 0; i < SPI2_POWER_MAX_RAILS; i++) {
        spi2_power_entry_t *e = &s_rails[i];
        if (!e->in_use || e->policy != SPI2_POWER_SHARED) {
            continue;
        }
        bool needed = e->need_fn ? e->need_fn() : true;
        axp2101_enable_rail(s_pmu, e->rail, needed);
    }
}

esp_err_t spi2_power_hold(axp2101_rail_t owned_rail)
{
    if (!s_mux) {
        return ESP_ERR_INVALID_STATE;
    }
    xSemaphoreTake(s_mux, portMAX_DELAY);
    /* Raise whatever this hold needs regardless of whether it's the first
     * holder overall - a later, concurrent hold can still be the one that
     * turns a given rail on for the first time (e.g. NFC opening a session
     * while SD already holds the bus for an unrelated reason). */
    bool settle_needed = raise_rails(owned_rail);
    s_holders++;
    xSemaphoreGive(s_mux);

    /* Delay outside the lock so a settling rail doesn't block unrelated
     * hold()/release() calls from other tasks. */
    if (settle_needed) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    return ESP_OK;
}

void spi2_power_release(axp2101_rail_t owned_rail)
{
    (void)owned_rail;   /* OWNED rails are never lowered by release() - see spi2_power.h */
    if (!s_mux) {
        return;
    }
    xSemaphoreTake(s_mux, portMAX_DELAY);
    if (s_holders > 0) {
        s_holders--;
    }
    if (s_holders == 0) {
        lower_rails();
    }
    xSemaphoreGive(s_mux);
}
