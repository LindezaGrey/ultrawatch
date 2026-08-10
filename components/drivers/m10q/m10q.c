/*
 * m10q.c - u-blox MIA-M10Q GNSS receiver (UART, NMEA-0183).
 *
 * A UART RX task parses NMEA sentences: GGA (position/altitude/sats/HDOP),
 * RMC (validity/speed/course/time) and GSV (per-satellite azimuth/elevation/
 * SNR, multi-sentence per constellation). The receiver is powered via the
 * AXP2101 BLDO1 rail only on demand; the always-on VRTC backup keeps its RTC
 * and ephemeris alive so power-ups are warm/hot starts.
 */
#include "m10q.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "axp2101.h"

static const char *TAG = "m10q";

#define M10Q_UART_NUM     UART_NUM_1
#define M10Q_BAUD         9600
#define M10Q_RX_BUF       512
#define M10Q_TX_BUF       0
#define M10Q_EVT_QUEUE    16
#define M10Q_RX_TASK_STACK 3072
#define M10Q_LINE_MAX     128

static i2c_master_dev_handle_t s_pmu;
static bool s_powered;
static m10q_state_t s_state = M10Q_STATE_OFF;
static m10q_fix_t s_fix;
static QueueHandle_t s_uart_queue;

/* GSV sentence merging: GSV frames arrive as N sentences per constellation;
 * satellites accumulate until the count is reached or a timeout resets. */
static uint8_t s_gsv_sent_expected;   /* total sentences in current frame */
static uint8_t s_gsv_sent_seen;
static uint8_t s_gsv_sats_expected;   /* total satellites in view */
static uint16_t s_gsv_sat_count;
static uint32_t s_gsv_last_ms;

/* ---- UBX helpers (for soft-standby on power-down) ---- */

static void ubx_send(uint8_t cls, uint8_t id, const uint8_t *payload, size_t len)
{
    if (!s_powered) {
        return;
    }
    uint8_t frame[M10Q_LINE_MAX + 8];
    size_t n = 0;
    frame[n++] = 0xB5;
    frame[n++] = 0x62;
    frame[n++] = cls;
    frame[n++] = id;
    frame[n++] = (uint8_t)(len & 0xFF);
    frame[n++] = (uint8_t)((len >> 8) & 0xFF);
    memcpy(frame + n, payload, len);
    n += len;
    uint8_t ck_a = 0, ck_b = 0;
    for (size_t i = 2; i < n; i++) {
        ck_a += frame[i];
        ck_b += ck_a;
    }
    frame[n++] = ck_a;
    frame[n++] = ck_b;
    uart_write_bytes(M10Q_UART_NUM, frame, n);
}

/* UBX-CFG-PM2 (0x06,0x3B): cyclic power save (on/off with acquisition
 * tracking). */
static void ubx_cfg_pm2(void)
{
    uint8_t pm2[44];
    memset(pm2, 0, sizeof(pm2));
    pm2[0] = 0x00;            /* version */
    pm2[1] = 0x00;
    pm2[4] = 0x01;            /* power setup request */
    pm2[6] = 0x00;            /* no flags */
    /* update/search periods, grid offset, on time, min acq time = 0 (default) */
    ubx_send(0x06, 0x3B, pm2, sizeof(pm2));
}

/* UBX-RXM-PMREQ (0x06,0x41): request software standby (keeps backup RAM). */
static void ubx_rxm_pmreq(void)
{
    uint8_t payload[8];
    memset(payload, 0, sizeof(payload));
    payload[4] = 0x02;        /* flags: backup */
    ubx_send(0x06, 0x41, payload, sizeof(payload));
}

/* ---- NMEA line parsing ---- */

static char *nmea_field(char *line, int idx)
{
    char *p = line;
    int i = 0;
    while (i < idx) {
        p = strchr(p, ',');
        if (!p) {
            return NULL;
        }
        p++;
        i++;
    }
    char *end = strchr(p, ',');
    if (end) {
        *end = '\0';
    }
    return p;
}

static int nmea_int(char *s, int *out)
{
    if (!s || !*s) {
        return -1;
    }
    char *end = NULL;
    long v = strtol(s, &end, 10);
    if (end == s) {
        return -1;
    }
    *out = (int)v;
    return 0;
}

static int nmea_double(char *s, double *out)
{
    if (!s || !*s) {
        return -1;
    }
    char *end = NULL;
    double v = strtod(s, &end);
    if (end == s) {
        return -1;
    }
    *out = v;
    return 0;
}

/* Convert ddmm.mmmm -> decimal degrees. */
static double nmea_latlon(double v, char hemi)
{
    double deg = (int)(v / 100.0);
    double min = v - deg * 100.0;
    double d = deg + min / 60.0;
    if (hemi == 'S' || hemi == 'W') {
        d = -d;
    }
    return d;
}

static void parse_gga(char *line)
{
    /* $GxGGA,hhmmss.ss,lat,N,lon,E,quality,sats,hdop,alt,M,... */
    double t, lat, lon, alt, hdop;
    int quality = 0, sats = 0;
    if (nmea_double(nmea_field(line, 1), &t) < 0) return;
    if (nmea_double(nmea_field(line, 2), &lat) < 0) return;
    char *ns = nmea_field(line, 3);
    if (nmea_double(nmea_field(line, 4), &lon) < 0) return;
    char *ew = nmea_field(line, 5);
    if (nmea_int(nmea_field(line, 6), &quality) < 0) return;
    if (nmea_int(nmea_field(line, 7), &sats) < 0) return;
    if (nmea_double(nmea_field(line, 8), &hdop) < 0) return;
    if (nmea_double(nmea_field(line, 9), &alt) < 0) return;
    if (!ns || !ew) return;

    s_fix.hdop = (uint16_t)(hdop * 10.0);
    s_fix.lat = nmea_latlon(lat, ns[0]);
    s_fix.lon = nmea_latlon(lon, ew[0]);
    s_fix.alt_m = alt;
    s_fix.sat_count = (uint16_t)sats;
    s_fix.hacc_m = (uint16_t)(hdop * M10Q_UERE_M);
    int hms = (int)t;
    s_fix.hour = (uint8_t)(hms / 10000);
    s_fix.minute = (uint8_t)((hms / 100) % 100);
    s_fix.second = (uint8_t)(hms % 100);
    s_fix.valid = (quality > 0);
    s_state = s_fix.valid ? M10Q_STATE_FIXED : M10Q_STATE_ACQUIRING;
}

static void parse_rmc(char *line)
{
    /* $GxRMC,hhmmss.ss,A,lat,N,lon,E,speed,course,date,... */
    char *f = nmea_field(line, 2);
    if (!f || f[0] != 'A') {
        return;   /* not valid */
    }
    double speed = 0, course = 0;
    if (nmea_double(nmea_field(line, 7), &speed) < 0) return;
    nmea_double(nmea_field(line, 8), &course);
    /* knots -> km/h */
    s_fix.speed_kmh = (uint16_t)(speed * 1.852 + 0.5);
    s_fix.course_deg = (uint16_t)(course + 0.5) % 360;
    s_fix.valid = true;
    s_state = M10Q_STATE_FIXED;
}

static void parse_gsv(char *line)
{
    /* $GxGSV,num_msgs,msg_num,sats_in_view,{prn,el,az,snr}*4 */
    int total_msgs = 0, msg_num = 0, sats_in_view = 0;
    if (nmea_int(nmea_field(line, 1), &total_msgs) < 0) return;
    if (nmea_int(nmea_field(line, 2), &msg_num) < 0) return;
    if (nmea_int(nmea_field(line, 3), &sats_in_view) < 0) return;

    uint32_t now = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    if (msg_num == 1 || now - s_gsv_last_ms > 2000) {
        s_gsv_sent_expected = (uint8_t)total_msgs;
        s_gsv_sats_expected = (uint8_t)sats_in_view;
        s_gsv_sat_count = 0;
        s_fix.sat_in_view = (uint16_t)sats_in_view;
    }
    s_gsv_last_ms = now;
    s_gsv_sent_seen = (uint8_t)msg_num;

    for (int i = 0; i < 4; i++) {
        int base = 4 + i * 4;
        char *prn_s = nmea_field(line, base);
        if (!prn_s || !*prn_s) {
            break;
        }
        int prn = 0, el = 0, az = 0, snr = 0;
        if (nmea_int(prn_s, &prn) < 0) break;
        nmea_int(nmea_field(line, base + 1), &el);
        nmea_int(nmea_field(line, base + 2), &az);
        if (nmea_int(nmea_field(line, base + 3), &snr) < 0) {
            snr = -1;
        }
        if (s_gsv_sat_count < M10Q_MAX_SATS) {
            m10q_sat_t *s = &s_fix.sats[s_gsv_sat_count];
            s->prn = (uint8_t)prn;
            s->elevation_deg = (int16_t)el;
            s->azimuth_deg = (int16_t)az;
            s->snr_db = (int16_t)snr;
            s->used = false;
        }
        s_gsv_sat_count++;
    }
    if (s_gsv_sent_seen >= s_gsv_sent_expected) {
        s_fix.sat_in_view = s_gsv_sats_expected;
    }
}

static void parse_line(char *line)
{
    if (line[0] != '$') {
        return;
    }
    if (strstr(line, "GGA,")) {
        parse_gga(line);
    } else if (strstr(line, "RMC,")) {
        parse_rmc(line);
    } else if (strstr(line, "GSV,")) {
        parse_gsv(line);
    }
}

static void uart_rx_task(void *arg)
{
    (void)arg;
    uart_event_t evt;
    char line[M10Q_LINE_MAX];
    size_t line_len = 0;
    for (;;) {
        if (xQueueReceive(s_uart_queue, &evt, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        if (evt.type == UART_DATA) {
            uint8_t buf[256];
            int n = uart_read_bytes(M10Q_UART_NUM, buf, sizeof(buf), pdMS_TO_TICKS(20));
            for (int i = 0; i < n; i++) {
                char c = (char)buf[i];
                if (c == '\n') {
                    if (line_len > 0) {
                        line[line_len] = '\0';
                        parse_line(line);
                        line_len = 0;
                    }
                } else if (c != '\r' && line_len < M10Q_LINE_MAX - 1) {
                    line[line_len++] = c;
                }
            }
        } else if (evt.type == UART_FIFO_OVF || evt.type == UART_BUFFER_FULL) {
            uart_flush_input(M10Q_UART_NUM);
        }
    }
}

esp_err_t m10q_init(i2c_master_dev_handle_t pmu)
{
    s_pmu = pmu;
    s_powered = false;
    s_state = M10Q_STATE_OFF;
    memset(&s_fix, 0, sizeof(s_fix));
    return ESP_OK;
}

esp_err_t m10q_power(bool on)
{
    if (on == s_powered) {
        return ESP_OK;
    }
    if (on) {
        uart_config_t cfg = {
            .baud_rate = M10Q_BAUD,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .source_clk = UART_SCLK_DEFAULT,
        };
        ESP_RETURN_ON_ERROR(uart_param_config(M10Q_UART_NUM, &cfg), TAG, "uart param");
        ESP_RETURN_ON_ERROR(uart_set_pin(M10Q_UART_NUM, M10Q_PIN_TX, M10Q_PIN_RX, -1, -1),
                            TAG, "uart pin");
        ESP_RETURN_ON_ERROR(uart_driver_install(M10Q_UART_NUM, M10Q_RX_BUF, M10Q_TX_BUF,
                                                M10Q_EVT_QUEUE, &s_uart_queue, 0),
                            TAG, "uart install");

        /* Power the receiver rail, then start the RX task. */
        axp2101_enable_rail(s_pmu, AXP2101_BLDO1, true);
        vTaskDelay(pdMS_TO_TICKS(100));
        s_powered = true;
        s_state = M10Q_STATE_ACQUIRING;
        memset(&s_fix, 0, sizeof(s_fix));
        xTaskCreate(uart_rx_task, "m10q_rx", M10Q_RX_TASK_STACK, NULL, 6, NULL);
        ubx_cfg_pm2();
        ESP_LOGI(TAG, "powered on (BLDO1)");
    } else {
        ubx_rxm_pmreq();   /* soft standby keeps backup RAM */
        vTaskDelay(pdMS_TO_TICKS(50));
        uart_driver_delete(M10Q_UART_NUM);
        axp2101_enable_rail(s_pmu, AXP2101_BLDO1, false);
        s_powered = false;
        s_state = M10Q_STATE_OFF;
        ESP_LOGI(TAG, "powered off (backup RAM kept)");
    }
    return ESP_OK;
}

esp_err_t m10q_get_fix(m10q_fix_t *fix)
{
    if (!fix) {
        return ESP_ERR_INVALID_ARG;
    }
    *fix = s_fix;
    return ESP_OK;
}

m10q_state_t m10q_get_state(void)
{
    return s_state;
}
