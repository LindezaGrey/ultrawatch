/*
 * sx1262.c - Semtech SX1262 LoRa transceiver driver.
 *
 * Command opcodes, register addresses, and the BUSY/command-ordering
 * protocol below are verified against the Semtech SX1261/2 datasheet
 * (Rev 1.2, DS.SX1261-2.W.APP, June 2019), sections 8 (Digital Interface),
 * 11 (List of Commands), 13 (Commands Interface) and 14.3 (Circuit
 * Configuration for Basic Rx Operation).
 *
 * Chip-generic only - no Meshtastic knowledge here. See meshtastic_radio.c
 * for the LongFast/EU_868 preset that calls sx1262_configure_lora().
 */
#include "sx1262.h"
#include "esp_check.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdlib.h>
#include <string.h>

static const char *TAG = "sx1262";

/* ---- opcodes (datasheet Table 11-1..11-5) ---- */
#define OP_SET_STANDBY           0x80
#define OP_SET_PACKET_TYPE       0x8A
#define OP_SET_RF_FREQUENCY      0x86
#define OP_SET_BUFFER_BASE_ADDR  0x8F
#define OP_SET_MODULATION_PARAMS 0x8B
#define OP_SET_PACKET_PARAMS     0x8C
#define OP_SET_DIO_IRQ_PARAMS    0x08
#define OP_SET_RX                0x82
#define OP_GET_IRQ_STATUS        0x12
#define OP_CLEAR_IRQ_STATUS      0x02
#define OP_GET_RX_BUFFER_STATUS  0x13
#define OP_GET_PACKET_STATUS     0x14
#define OP_READ_BUFFER           0x1E
#define OP_WRITE_REGISTER        0x0D
#define OP_GET_STATUS            0xC0
#define OP_GET_RSSI_INST         0x15
#define OP_GET_DEVICE_ERRORS     0x17
#define OP_CLEAR_DEVICE_ERRORS   0x07
#define OP_SET_DIO3_TCXO_CTRL    0x97
#define OP_SET_DIO2_RF_SWITCH    0x9D

/* This board's SX1262 module (schematic: "HPB16B3", no external crystal
 * near it - unlike the ESP32/RTC crystals shown elsewhere on the same
 * sheet) uses a TCXO powered via DIO3, and DIO2 drives an antenna TX/RX
 * switch (a SKY13453 next to the antenna connector). Both confirmed
 * against LilyGO's own official reference firmware for this exact board
 * (LilyGoLib's examples/radio/SX1262/SX126x_Receive.ino:
 * radio.setTCXO(3.0); radio.setDio2AsRfSwitch();). Without the TCXO
 * config the chip has no working 32MHz reference for any frequency-
 * dependent operation - this is the root cause of a cold-boot
 * XOSC_START_ERR and a receiver that never picks up real RF energy. */
#define TCXO_VOLTAGE_3V0 0x06     /* datasheet Table 13-35 */
#define TCXO_DELAY_STEPS 320      /* 320 * 15.625us = 5ms, matches RadioLib's default */

/* ---- registers (Table 12-1) ---- */
#define REG_LORA_SYNC_WORD_MSB 0x0740
#define REG_RX_GAIN             0x08AC
#define RX_GAIN_BOOSTED         0x96

/* ---- IRQ bits (Table 8-4) ---- */
#define IRQ_RXDONE   (1u << 1)
#define IRQ_CRC_ERR  (1u << 6)
#define IRQ_TIMEOUT  (1u << 9)

#define BUSY_WAIT_ATTEMPTS 200
#define BUSY_WAIT_STEP_MS  1

/* Largest real command: opcode + SetPacketParams' 9 params. */
#define CMD_MAX_LEN 12

static spi_device_handle_t s_spi;
static bool s_rx_active;

/* Task to notify when DIO1 (RxDone) fires - set at the top of every
 * sx1262_recv() call, read from ISR context. A single pointer-sized write/
 * read is atomic on this target, and there is exactly one caller (mesh_log's
 * background listener) for the lifetime of the process, so no lock is
 * needed. NULL-checked in the ISR: DIO1 can only assert after SetRx, which
 * the caller always issues before its first sx1262_recv() call, but a
 * packet arriving in that narrow window would otherwise notify a NULL
 * handle. Harmless either way - sx1262_recv() also checks the GPIO level
 * directly on entry, and DIO1 stays asserted until ClearIrqStatus, so a
 * missed notification there still gets caught by that level check. */
static TaskHandle_t s_recv_task;

/* ---- BUSY protocol + raw SPI transaction primitives ---- */

/* Bounded attempt-loop, modeled on bhi260ap_drain_wakeup_fifo()'s shape. */
static esp_err_t busy_wait(void)
{
    for (int i = 0; i < BUSY_WAIT_ATTEMPTS; i++) {
        if (gpio_get_level(SX1262_PIN_BUSY) == 0) {
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(BUSY_WAIT_STEP_MS));
    }
    return ESP_ERR_TIMEOUT;
}

/* "Write" command: opcode + params, one transaction, BUSY re-asserted on
 * completion while the chip processes it - wait for it to clear again. */
static esp_err_t cmd_write(uint8_t opcode, const uint8_t *params, size_t param_len)
{
    if (param_len > CMD_MAX_LEN - 1) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = busy_wait();
    if (err != ESP_OK) {
        return err;
    }

    uint8_t tx[CMD_MAX_LEN];
    tx[0] = opcode;
    if (param_len > 0) {
        memcpy(tx + 1, params, param_len);
    }

    spi_transaction_t t = {
        .length = (size_t)(1 + param_len) * 8,
        .tx_buffer = tx,
    };
    err = spi_device_polling_transmit(s_spi, &t);
    if (err != ESP_OK) {
        return err;
    }
    return busy_wait();
}

/* "Read" command: opcode + NOPs, response arrives shifted out one byte per
 * NOP sent. rx[0] is always RFU (opcode-echo slot); `skip` further bytes
 * (RFU + a Status byte, or just RFU alone for GetStatus itself) are
 * discarded before the wanted `out_len` bytes. No BUSY-wait after - read
 * commands don't assert it (datasheet SS8.3.1). */
static esp_err_t cmd_read(uint8_t opcode, size_t skip, uint8_t *out, size_t out_len)
{
    size_t total = skip + out_len;
    if (total > CMD_MAX_LEN) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = busy_wait();
    if (err != ESP_OK) {
        return err;
    }

    uint8_t tx[CMD_MAX_LEN] = { 0 };
    uint8_t rx[CMD_MAX_LEN] = { 0 };
    tx[0] = opcode;

    spi_transaction_t t = {
        .length = total * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    err = spi_device_polling_transmit(s_spi, &t);
    if (err != ESP_OK) {
        return err;
    }
    memcpy(out, rx + skip, out_len);
    return ESP_OK;
}

static esp_err_t write_register(uint16_t addr, const uint8_t *data, size_t len)
{
    uint8_t p[2 + 4];
    if (len > sizeof(p) - 2) {
        return ESP_ERR_INVALID_ARG;
    }
    p[0] = (uint8_t)(addr >> 8);
    p[1] = (uint8_t)addr;
    memcpy(p + 2, data, len);
    return cmd_write(OP_WRITE_REGISTER, p, 2 + len);
}

/* ReadBuffer has its own shape (Table 13-27), not cmd_read's: opcode +
 * offset + 2 status bytes + payload. Real data starts at rx index 3, not
 * the skip-2 pattern the Get* status commands use. Heap-allocated since
 * payload can be up to 255 bytes - too large for a routine stack frame. */
static esp_err_t read_buffer(uint8_t offset, uint8_t *buf, uint8_t len)
{
    esp_err_t err = busy_wait();
    if (err != ESP_OK) {
        return err;
    }

    size_t total = 2 + 2 + (size_t)len;
    uint8_t *tx = calloc(1, total);
    uint8_t *rx = calloc(1, total);
    if (!tx || !rx) {
        free(tx);
        free(rx);
        return ESP_ERR_NO_MEM;
    }
    tx[0] = OP_READ_BUFFER;
    tx[1] = offset;

    spi_transaction_t t = {
        .length = total * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    err = spi_device_polling_transmit(s_spi, &t);
    if (err == ESP_OK) {
        memcpy(buf, rx + 3, len);
    }
    free(tx);
    free(rx);
    return err;
}

static void reset_pulse(void)
{
    gpio_set_level(SX1262_PIN_RESET, 0);
    vTaskDelay(pdMS_TO_TICKS(1));   /* >=100us required; 1 tick is safely longer */
    gpio_set_level(SX1262_PIN_RESET, 1);
    busy_wait();                    /* chip auto-calibrates, lands in STDBY_RC */
}

/* ---- one function per opcode actually used ---- */

static esp_err_t set_standby_rc(void)
{
    uint8_t p[1] = { 0x00 };
    return cmd_write(OP_SET_STANDBY, p, sizeof(p));
}

static esp_err_t set_packet_type_lora(void)
{
    uint8_t p[1] = { 0x01 };
    return cmd_write(OP_SET_PACKET_TYPE, p, sizeof(p));
}

static uint32_t hz_to_rf_freq(uint32_t freq_hz)
{
    /* RfFreq = round(freq_hz * 2^25 / Fxtal), Fxtal = 32 MHz. */
    uint64_t v = ((uint64_t)freq_hz * 33554432ULL + 16000000ULL) / 32000000ULL;
    return (uint32_t)v;
}

esp_err_t sx1262_set_frequency(uint32_t freq_hz)
{
    uint32_t rf = hz_to_rf_freq(freq_hz);
    uint8_t p[4] = {
        (uint8_t)(rf >> 24), (uint8_t)(rf >> 16), (uint8_t)(rf >> 8), (uint8_t)rf,
    };
    return cmd_write(OP_SET_RF_FREQUENCY, p, sizeof(p));
}

static esp_err_t set_buffer_base_address(uint8_t tx_base, uint8_t rx_base)
{
    uint8_t p[2] = { tx_base, rx_base };
    return cmd_write(OP_SET_BUFFER_BASE_ADDR, p, sizeof(p));
}

static esp_err_t set_modulation_params(uint8_t sf, uint8_t bw, uint8_t cr, bool ldro)
{
    uint8_t p[8] = { sf, bw, cr, (uint8_t)(ldro ? 1 : 0), 0, 0, 0, 0 };
    return cmd_write(OP_SET_MODULATION_PARAMS, p, sizeof(p));
}

static esp_err_t set_packet_params(uint16_t preamble_len, bool explicit_header,
                                   uint8_t payload_len_max, bool crc_on, bool invert_iq)
{
    uint8_t p[9] = {
        (uint8_t)(preamble_len >> 8), (uint8_t)preamble_len,
        explicit_header ? 0x00 : 0x01,   /* 0x00=explicit(variable), 0x01=implicit(fixed) */
        payload_len_max,
        crc_on ? 0x01 : 0x00,
        invert_iq ? 0x01 : 0x00,
        0, 0, 0,
    };
    return cmd_write(OP_SET_PACKET_PARAMS, p, sizeof(p));
}

static esp_err_t set_dio_irq_params(void)
{
    uint16_t mask = IRQ_RXDONE | IRQ_TIMEOUT;
    uint8_t p[8] = {
        (uint8_t)(mask >> 8), (uint8_t)mask,   /* IrqMask */
        (uint8_t)(mask >> 8), (uint8_t)mask,   /* Dio1Mask */
        0, 0,                                   /* Dio2Mask */
        0, 0,                                   /* Dio3Mask */
    };
    return cmd_write(OP_SET_DIO_IRQ_PARAMS, p, sizeof(p));
}

/* Sync word nibble expansion, verified against the datasheet's own known
 * examples (Table 12-1): 0x34 (public) -> 0x3444, 0x12 (private) -> 0x1424.
 * Formula: msb = (sw & 0xF0) | 0x04, lsb = ((sw & 0x0F) << 4) | 0x04. */
static esp_err_t set_lora_sync_word(uint8_t sw)
{
    uint8_t data[2] = {
        (uint8_t)((sw & 0xF0) | 0x04),
        (uint8_t)(((sw & 0x0F) << 4) | 0x04),
    };
    return write_register(REG_LORA_SYNC_WORD_MSB, data, sizeof(data));
}

static esp_err_t set_dio3_tcxo_ctrl(uint8_t voltage_code, uint32_t delay_steps)
{
    uint8_t p[4] = {
        voltage_code,
        (uint8_t)(delay_steps >> 16), (uint8_t)(delay_steps >> 8), (uint8_t)delay_steps,
    };
    return cmd_write(OP_SET_DIO3_TCXO_CTRL, p, sizeof(p));
}

static esp_err_t set_dio2_rf_switch(bool enable)
{
    uint8_t p[1] = { enable ? 0x01 : 0x00 };
    return cmd_write(OP_SET_DIO2_RF_SWITCH, p, sizeof(p));
}

static esp_err_t set_boosted_rx_gain(void)
{
    uint8_t v = RX_GAIN_BOOSTED;
    return write_register(REG_RX_GAIN, &v, 1);
}

static esp_err_t set_rx(uint32_t timeout_steps)
{
    uint8_t p[3] = {
        (uint8_t)(timeout_steps >> 16), (uint8_t)(timeout_steps >> 8), (uint8_t)timeout_steps,
    };
    return cmd_write(OP_SET_RX, p, sizeof(p));
}

static esp_err_t get_irq_status(uint16_t *status)
{
    uint8_t out[2];
    esp_err_t err = cmd_read(OP_GET_IRQ_STATUS, 2, out, sizeof(out));
    if (err != ESP_OK) {
        return err;
    }
    *status = ((uint16_t)out[0] << 8) | out[1];
    return ESP_OK;
}

static esp_err_t clear_irq_status(uint16_t mask)
{
    uint8_t p[2] = { (uint8_t)(mask >> 8), (uint8_t)mask };
    return cmd_write(OP_CLEAR_IRQ_STATUS, p, sizeof(p));
}

static esp_err_t get_rx_buffer_status(uint8_t *payload_len, uint8_t *rx_start_ptr)
{
    uint8_t out[2];
    esp_err_t err = cmd_read(OP_GET_RX_BUFFER_STATUS, 2, out, sizeof(out));
    if (err != ESP_OK) {
        return err;
    }
    *payload_len = out[0];
    *rx_start_ptr = out[1];
    return ESP_OK;
}

static esp_err_t get_packet_status(int16_t *rssi_dbm, int8_t *snr_db)
{
    uint8_t out[3];   /* RssiPkt, SnrPkt, SignalRssiPkt (unused) */
    esp_err_t err = cmd_read(OP_GET_PACKET_STATUS, 2, out, sizeof(out));
    if (err != ESP_OK) {
        return err;
    }
    *rssi_dbm = (int16_t)(-(int16_t)out[0] / 2);
    *snr_db = (int8_t)((int8_t)out[1] / 4);   /* SnrPkt is two's-complement, actual = /4 dB */
    return ESP_OK;
}

static esp_err_t get_status(uint8_t *status_byte)
{
    return cmd_read(OP_GET_STATUS, 1, status_byte, 1);
}

/* ---- public API ---- */

esp_err_t sx1262_configure_lora(const sx1262_lora_params_t *params)
{
    if (!params) {
        return ESP_ERR_INVALID_ARG;
    }
    /* Order matters (datasheet SS14.4): PacketType first, then modulation
     * params, then packet params - "if this order is not respected, the
     * behavior of the device could be unexpected". */
    ESP_RETURN_ON_ERROR(set_standby_rc(), TAG, "standby");
    ESP_RETURN_ON_ERROR(set_dio3_tcxo_ctrl(TCXO_VOLTAGE_3V0, TCXO_DELAY_STEPS), TAG, "tcxo ctrl");
    ESP_RETURN_ON_ERROR(set_dio2_rf_switch(true), TAG, "dio2 rf switch");
    /* Clear the cold-start XOSC_START_ERR now that the chip knows to wait
     * for the TCXO (datasheet SS13.3.6: this flag is expected at POR when a
     * TCXO is used, before the chip is told about it - not a real fault). */
    sx1262_clear_device_errors();

    ESP_RETURN_ON_ERROR(set_packet_type_lora(), TAG, "packet type");
    ESP_RETURN_ON_ERROR(sx1262_set_frequency(params->freq_hz), TAG, "rf freq");
    ESP_RETURN_ON_ERROR(set_buffer_base_address(0x00, 0x00), TAG, "buffer base");
    ESP_RETURN_ON_ERROR(set_modulation_params(params->sf, params->bw, params->cr,
                                              params->low_data_rate_optimize),
                        TAG, "mod params");
    ESP_RETURN_ON_ERROR(set_packet_params(params->preamble_len, params->explicit_header,
                                          params->payload_len_max, params->crc_on,
                                          params->invert_iq),
                        TAG, "packet params");
    ESP_RETURN_ON_ERROR(set_dio_irq_params(), TAG, "dio irq params");
    ESP_RETURN_ON_ERROR(set_lora_sync_word(params->sync_word), TAG, "sync word");
    if (params->boosted_rx_gain) {
        ESP_RETURN_ON_ERROR(set_boosted_rx_gain(), TAG, "rx gain");
    }
    ESP_LOGI(TAG, "configured LoRa: %lu Hz SF%u bw=0x%02x cr=0x%02x sync=0x%02x",
             (unsigned long)params->freq_hz, params->sf, params->bw, params->cr,
             params->sync_word);
    return ESP_OK;
}

esp_err_t sx1262_rx_start(void)
{
    if (s_rx_active) {
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t err = set_rx(0xFFFFFF);   /* Rx Continuous - no hardware Timeout IRQ in this mode */
    if (err == ESP_OK) {
        s_rx_active = true;
    }
    return err;
}

esp_err_t sx1262_rx_stop(void)
{
    esp_err_t err = set_standby_rc();
    s_rx_active = false;
    return err;
}

/* DIO1 fires (rising edge) on RxDone - see set_dio_irq_params(). Just wakes
 * whoever is blocked in sx1262_recv(); all the real work (GetIrqStatus,
 * ClearIrqStatus, reading the packet) happens on that task, not here. */
static void IRAM_ATTR sx1262_dio1_isr(void *arg)
{
    (void)arg;
    if (s_recv_task) {
        BaseType_t woken = pdFALSE;
        vTaskNotifyGiveFromISR(s_recv_task, &woken);
        if (woken == pdTRUE) {
            portYIELD_FROM_ISR();
        }
    }
}

esp_err_t sx1262_recv(uint8_t *buf, size_t buf_cap, size_t *out_len,
                       int16_t *rssi_dbm, int8_t *snr_db, bool *crc_ok,
                       int timeout_ms)
{
    if (!buf || !out_len || !rssi_dbm || !snr_db || !crc_ok) {
        return ESP_ERR_INVALID_ARG;
    }
    s_recv_task = xTaskGetCurrentTaskHandle();

    TickType_t start = xTaskGetTickCount();
    TickType_t deadline = start + pdMS_TO_TICKS(timeout_ms);
    for (;;) {
        if (gpio_get_level(SX1262_PIN_IRQ) != 0) {
            uint16_t irq = 0;
            esp_err_t err = get_irq_status(&irq);
            if (err == ESP_OK && (irq & IRQ_RXDONE)) {
                *crc_ok = (irq & IRQ_CRC_ERR) == 0;
                clear_irq_status(irq);

                uint8_t payload_len = 0, rx_ptr = 0;
                err = get_rx_buffer_status(&payload_len, &rx_ptr);
                if (err != ESP_OK) {
                    return err;
                }
                if (payload_len > buf_cap) {
                    payload_len = (uint8_t)buf_cap;
                }
                err = read_buffer(rx_ptr, buf, payload_len);
                if (err != ESP_OK) {
                    return err;
                }
                *out_len = payload_len;
                int16_t rssi = 0;
                int8_t snr = 0;
                get_packet_status(&rssi, &snr);   /* best-effort, don't fail recv on this */
                *rssi_dbm = rssi;
                *snr_db = snr;
                return ESP_OK;
            } else if (err == ESP_OK) {
                clear_irq_status(irq);   /* clear whatever else fired (e.g. a lone CRC error) */
            }
        }
        TickType_t now = xTaskGetTickCount();
        if ((now - start) >= (TickType_t)pdMS_TO_TICKS(timeout_ms)) {
            return ESP_ERR_TIMEOUT;
        }
        /* Block until DIO1's ISR wakes us or the deadline arrives - no
         * polling. If a notification is already pending (DIO1 fired between
         * the previous call's ClearIrqStatus and this call re-arming, or
         * this is the very first call and it raced sx1262_rx_start()'s
         * SetRx), this returns immediately and the level check above
         * catches it on the next loop iteration. */
        ulTaskNotifyTake(pdTRUE, deadline - now);
    }
}

esp_err_t sx1262_init(spi_device_handle_t spi)
{
    s_spi = spi;

    gpio_config_t reset_io = {
        .pin_bit_mask = 1ULL << SX1262_PIN_RESET,
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&reset_io);
    gpio_set_level(SX1262_PIN_RESET, 1);

    gpio_config_t busy_io = {
        .pin_bit_mask = 1ULL << SX1262_PIN_BUSY,
        .mode = GPIO_MODE_INPUT,
    };
    gpio_config(&busy_io);

    gpio_config_t irq_io = {
        .pin_bit_mask = 1ULL << SX1262_PIN_IRQ,
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_POSEDGE,   /* DIO1 idles low, pulses high on RxDone */
    };
    gpio_config(&irq_io);

    /* The ISR service is shared across the whole app (touch/m10q/power_mgmt
     * also install it) - ESP_ERR_INVALID_STATE just means someone beat us to
     * it, which is fine; anything else means gpio_isr_handler_add() below
     * will fail too, so log and continue (matches this driver's log-and-
     * continue convention for every other init step). */
    esp_err_t isr_err = gpio_install_isr_service(0);
    if (isr_err != ESP_OK && isr_err != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "gpio_install_isr_service: %s", esp_err_to_name(isr_err));
    } else {
        gpio_isr_handler_add(SX1262_PIN_IRQ, sx1262_dio1_isr, NULL);
    }

    reset_pulse();

    esp_err_t err = set_standby_rc();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "SetStandby failed: %s", esp_err_to_name(err));
        return ESP_OK;   /* log-and-continue, matches every other board driver */
    }

    uint8_t status = 0;
    err = get_status(&status);
    if (err == ESP_OK) {
        uint8_t chip_mode = (status >> 4) & 0x07;
        ESP_LOGI(TAG, "SX1262 found, status=0x%02x chip_mode=0x%x (0x2=STDBY_RC expected)",
                 status, chip_mode);
    } else {
        ESP_LOGW(TAG, "GetStatus failed: %s", esp_err_to_name(err));
    }

    uint16_t errors = 0;
    if (sx1262_get_device_errors(&errors) == ESP_OK && errors != 0) {
        ESP_LOGW(TAG, "device errors at boot: 0x%04x (cleared)", errors);
        sx1262_clear_device_errors();
    }
    return ESP_OK;
}

esp_err_t sx1262_send(const uint8_t *buf, size_t len, int timeout_ms)
{
    (void)buf;
    (void)len;
    (void)timeout_ms;
    return ESP_ERR_NOT_SUPPORTED;   /* RX-only for now */
}

esp_err_t sx1262_get_rssi_inst(int16_t *rssi_dbm)
{
    uint8_t out;
    esp_err_t err = cmd_read(OP_GET_RSSI_INST, 2, &out, 1);
    if (err != ESP_OK) {
        return err;
    }
    *rssi_dbm = (int16_t)(-(int16_t)out / 2);
    return ESP_OK;
}

esp_err_t sx1262_get_device_errors(uint16_t *errors)
{
    uint8_t out[2];
    esp_err_t err = cmd_read(OP_GET_DEVICE_ERRORS, 2, out, sizeof(out));
    if (err != ESP_OK) {
        return err;
    }
    *errors = ((uint16_t)out[0] << 8) | out[1];
    return ESP_OK;
}

esp_err_t sx1262_clear_device_errors(void)
{
    uint8_t p[2] = { 0x00, 0x00 };
    return cmd_write(OP_CLEAR_DEVICE_ERRORS, p, sizeof(p));
}
