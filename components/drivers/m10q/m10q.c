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
#define M10Q_BAUD_INIT    38400   /* factory default of the MIA-M10Q */
#define M10Q_BAUD_RUN     115200  /* raised after probing (RAM-only) */
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
static uint32_t s_rx_bytes;      /* debug: bytes received since power-on */
static uint32_t s_nmea_lines;   /* debug: NMEA lines parsed since power-on */
static uint32_t s_gsv_count;    /* debug: GSV sentences seen */

/* GSV sentence merging: GSV frames arrive as N sentences per constellation;
 * satellites accumulate until the count is reached or a timeout resets. */
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

/* Probe the UART baud rate. The MIA-M10Q answers at 38400 (factory), 115200
 * (if a previous RAM config raised it and the module kept power) or 9600.
 * We detect a live receiver by watching for UBX sync bytes (0xB5 0x62) in
 * response to a UBX-MON-VER request. Returns the found baud or 0. */
static uint32_t m10q_probe_baud(void)
{
    static const uint32_t probe_bauds[] = { M10Q_BAUD_INIT, M10Q_BAUD_RUN, 9600 };
    for (size_t pb = 0; pb < sizeof(probe_bauds) / sizeof(probe_bauds[0]); pb++) {
        uart_set_baudrate(M10Q_UART_NUM, probe_bauds[pb]);
        vTaskDelay(pdMS_TO_TICKS(200));
        /* Ask for the module version; a live module answers with a UBX
         * frame. NMEA chatter is ignored. */
        for (int retry = 0; retry < 3; retry++) {
            uart_flush_input(M10Q_UART_NUM);
            uint8_t req[] = { 0xB5, 0x62, 0x0A, 0x04, 0x00, 0x00, 0x0E, 0x34 };
            uart_write_bytes(M10Q_UART_NUM, req, sizeof(req));
            uint32_t deadline = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS) + 1500;
            bool seen_sync = false;
            while ((uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS) < deadline) {
                uint8_t b = 0;
                if (uart_read_bytes(M10Q_UART_NUM, &b, 1, pdMS_TO_TICKS(50)) > 0) {
                    if (b == 0xB5) {
                        seen_sync = true;
                        break;
                    }
                }
            }
            if (seen_sync) {
                ESP_LOGI(TAG, "module responds at %lu baud", (unsigned long)probe_bauds[pb]);
                return probe_bauds[pb];
            }
        }
    }
    return 0;
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

#define NMEA_MAX_FIELDS 24

/* Split a NMEA line into comma-separated fields in-place (commas become NUL),
 * returning the number of fields. field[0] is the talker+type token. */
static int nmea_split(char *line, char **field, int max)
{
    int n = 0;
    char *p = line;
    while (n < max) {
        field[n++] = p;
        char *comma = strchr(p, ',');
        if (!comma) {
            break;
        }
        *comma = '\0';
        p = comma + 1;
    }
    return n;
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
    char *f[NMEA_MAX_FIELDS];
    int nf = nmea_split(line, f, NMEA_MAX_FIELDS);
    if (nf < 10) return;
    double t, lat, lon, alt, hdop;
    int quality = 0, sats = 0;
    if (nmea_double(f[1], &t) < 0) return;
    if (nmea_double(f[2], &lat) < 0) return;
    if (nmea_double(f[4], &lon) < 0) return;
    if (nmea_int(f[6], &quality) < 0) return;
    if (nmea_int(f[7], &sats) < 0) return;
    if (nmea_double(f[8], &hdop) < 0) return;
    if (nmea_double(f[9], &alt) < 0) return;
    if (!f[3] || !f[5]) return;

    s_fix.hdop = (uint16_t)(hdop * 10.0);
    s_fix.lat = nmea_latlon(lat, f[3][0]);
    s_fix.lon = nmea_latlon(lon, f[5][0]);
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
    char *f[NMEA_MAX_FIELDS];
    int nf = nmea_split(line, f, NMEA_MAX_FIELDS);
    if (nf < 9 || !f[2] || f[2][0] != 'A') {
        return;   /* not valid */
    }
    double speed = 0, course = 0;
    if (nmea_double(f[7], &speed) < 0) return;
    nmea_double(f[8], &course);
    /* knots -> km/h */
    s_fix.speed_kmh = (uint16_t)(speed * 1.852 + 0.5);
    s_fix.course_deg = (uint16_t)(course + 0.5) % 360;
    s_fix.valid = true;
    s_state = M10Q_STATE_FIXED;
}

static void parse_gsv(char *line)
{
    /* $GxGSV,num_msgs,msg_num,sats_in_view,{prn,el,az,snr}*4
     * The receiver emits a separate multi-sentence GSV frame per constellation
     * (GP/GL/GA/GB). Accumulate satellites across all of them into sats[] and
     * refresh the accumulator after a gap (a full sky scan). */
    char *f[NMEA_MAX_FIELDS];
    int nf = nmea_split(line, f, NMEA_MAX_FIELDS);
    if (nf < 4) return;
    /* f[1]=num_msgs, f[2]=msg_num, f[3]=sats_in_view. The satellite data
     * starts at field 4; validate structure by requiring enough fields. */

    uint32_t now = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    if (now - s_gsv_last_ms > 2000) {
        /* New sky scan: restart the accumulator. */
        s_gsv_sat_count = 0;
        s_fix.sat_in_view = 0;
    }
    s_gsv_last_ms = now;

    for (int i = 0; i < 4; i++) {
        int base = 4 + i * 4;
        if (base + 3 >= nf) {
            break;
        }
        int prn = 0, el = 0, az = 0, snr = 0;
        if (nmea_int(f[base], &prn) < 0) break;
        nmea_int(f[base + 1], &el);
        nmea_int(f[base + 2], &az);
        if (nmea_int(f[base + 3], &snr) < 0) {
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
        if (s_fix.sat_in_view < M10Q_MAX_SATS) {
            s_fix.sat_in_view = s_gsv_sat_count;
        }
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
        s_gsv_count++;
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
        if (!s_powered || !s_uart_queue) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        if (xQueueReceive(s_uart_queue, &evt, pdMS_TO_TICKS(200)) != pdTRUE) {
            continue;
        }
        if (evt.type == UART_DATA) {
            uint8_t buf[256];
            int n = uart_read_bytes(M10Q_UART_NUM, buf, sizeof(buf), pdMS_TO_TICKS(20));
            s_rx_bytes += (uint32_t)n;
            for (int i = 0; i < n; i++) {
                char c = (char)buf[i];
                if (c == '\n') {
                    if (line_len > 0) {
                        line[line_len] = '\0';
                        s_nmea_lines++;
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
    /* Persistent RX task; it blocks on the UART event queue (created when the
     * UART is installed on power-on) and survives power-off. */
    xTaskCreate(uart_rx_task, "m10q_rx", M10Q_RX_TASK_STACK, NULL, 6, NULL);
    return ESP_OK;
}

esp_err_t m10q_power(bool on)
{
    if (on == s_powered) {
        return ESP_OK;
    }
    if (on) {
        uart_config_t cfg = {
            .baud_rate = M10Q_BAUD_INIT,
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

        /* Power the receiver rail and let it boot. */
        axp2101_enable_rail(s_pmu, AXP2101_BLDO1, true);
        vTaskDelay(pdMS_TO_TICKS(500));

        s_powered = true;
        uint32_t baud = m10q_probe_baud();
        if (baud == 0) {
            ESP_LOGW(TAG, "no UBX answer at 38400/115200/9600; using NMEA at 38400");
            baud = M10Q_BAUD_INIT;
        } else if (baud != M10Q_BAUD_RUN) {
            /* Raise the UART speed (RAM-only config, re-applied each boot).
             * The module switches after the ACK. */
            uint8_t valset[] = {
                0x00, 0x01, 0x00, 0x00,                       /* version, layer RAM */
                0x01, 0x00, 0x52, 0x40,                       /* CFG_UART1_BAUDRATE */
                M10Q_BAUD_RUN & 0xFF, (M10Q_BAUD_RUN >> 8) & 0xFF,
                (M10Q_BAUD_RUN >> 16) & 0xFF, (M10Q_BAUD_RUN >> 24) & 0xFF,
            };
            ubx_send(0x06, 0x8A, valset, sizeof(valset));   /* CFG-VALSET */
            vTaskDelay(pdMS_TO_TICKS(100));
            uart_flush_input(M10Q_UART_NUM);
            uart_set_baudrate(M10Q_UART_NUM, M10Q_BAUD_RUN);
            ESP_LOGI(TAG, "UART raised to %lu baud", (unsigned long)M10Q_BAUD_RUN);
        }

        s_rx_bytes = 0;
        s_nmea_lines = 0;
        s_state = M10Q_STATE_ACQUIRING;
        memset(&s_fix, 0, sizeof(s_fix));
        ubx_cfg_pm2();
        ESP_LOGI(TAG, "powered on (BLDO1) at %lu baud", (unsigned long)baud);
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

void m10q_get_dbg(uint32_t *rx_bytes, uint32_t *nmea_lines)
{
    if (rx_bytes) {
        *rx_bytes = s_rx_bytes;
    }
    if (nmea_lines) {
        *nmea_lines = s_nmea_lines;
    }
}

uint32_t m10q_get_gsv_count(void)
{
    return s_gsv_count;
}
