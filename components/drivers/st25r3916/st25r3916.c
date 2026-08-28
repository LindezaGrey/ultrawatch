/*
 * st25r3916.c - ST ST25R3916 NFC/HF reader driver (ISO14443-A, reader mode).
 *
 * SPI framing, direct-command codes, and register addresses below are
 * verified against the ST25R3916/7 datasheet (DS12484 Rev 8), sections
 * 4.3.3 (Serial peripheral interface), 4.3.1 (Interrupt interface), 4.4
 * (Direct commands), and 4.5 (Register description).
 *
 * Unlike the SX1262, this chip has no BUSY pin - the SPI transaction itself
 * is the synchronization point, and command completion is signalled via
 * the Main interrupt register (self-clearing on read, datasheet SS4.3.1)
 * instead of a GPIO level.
 *
 * "Automatic anti-collision including SAK" (SS4.2.14, NFCIP-1 passive
 * target definition register) only applies when this chip acts as a
 * passive target (card emulation) - the opposite of what this driver does.
 * In reader/initiator mode the chip only provides low-level assistance
 * (the antcl bit switches TX into bit-oriented anticollision framing, and
 * the Collision display register reports where a real collision happened);
 * the ISO14443-A REQA -> ATQA -> SEL_CLx cascade itself is implemented
 * here, same as it would be against any other bare RF front-end. Single-tag
 * only - a genuine multi-tag collision surfaces as a timeout or a garbled
 * read rather than being resolved.
 *
 * The raw datasheet bring-up above is NOT sufficient for the chip to
 * actually radiate a usable field or receive a real tag's backscatter -
 * every real reader also needs board-level analog tuning (TX driver
 * resistance/overshoot-undershoot protection, receiver gain/AGC, the
 * correlator, antenna trim, field-detector thresholds, ...), none of which
 * has a single "correct" datasheet default since it depends on the
 * antenna/PCB. apply_analog_defaults() below applies LilyGo's own actual
 * values, extracted from the analog configuration table ST's official RFAL
 * middleware ships (this is genuinely what LilyGo's firmware uses under the
 * hood - LilyGoLib's examples/factory/app_nfc.cpp wraps RfalNfcClass from
 * https://github.com/stm32duino/ST25R3916, the same Arduino port of RFAL;
 * these are the exact values in that library's
 * src/rfal_rfst25r3916_analogConfigTbl.h, narrowed to just the entries this
 * driver's NFC-A/106kbit-only, OOK-modulation-only reader path needs -
 * RFAL's own AM-modulation and other-bitrate entries are dropped since they
 * get overridden back to these same OOK/106k values in RFAL's own sequencing
 * whenever this driver's use case is what's active).
 */
#include "st25r3916.h"
#include "axp2101.h"
#include "esp_check.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <string.h>

static const char *TAG = "st25r3916";

/* ---- register addresses (space A, datasheet SS4.5) ---- */
#define REG_IO_CONF2       0x01
#define REG_OP_CONTROL     0x02
#define REG_MODE           0x03
#define REG_BITRATE        0x04
#define REG_ISO14443A_NFC  0x05
#define REG_MAIN_IRQ       0x1A
#define REG_FIFO_STATUS1   0x1E
#define REG_FIFO_STATUS2   0x1F
#define REG_NUM_TX_BYTES1  0x22
#define REG_NUM_TX_BYTES2  0x23
#define REG_AUX_DISPLAY    0x31
#define REG_IC_IDENTITY    0x3F

/* ---- register addresses used only by apply_analog_defaults() below (space
 * A unless marked "space B" - space B needs the 0xFB prefix, see
 * reg_write_b()/reg_read_b()). Addresses and bit values cross-checked
 * against stm32duino/ST25R3916's rfal_rfst25r3916_analogConfigTbl.h, the
 * same values LilyGo's own firmware applies via that library. ---- */
#define REG_IO_CONF1            0x00
#define REG_PASSIVE_TARGET      0x08
#define REG_RX_CONF1            0x0B
#define REG_RX_CONF2            0x0C
#define REG_RX_CONF3            0x0D
#define REG_RX_CONF4            0x0E
#define REG_AUX                 0x0A
#define REG_ANT_TUNE_A          0x26
#define REG_ANT_TUNE_B          0x27
#define REG_TX_DRIVER           0x28
#define REG_FIELD_THRESHOLD_ACTV   0x2A
#define REG_FIELD_THRESHOLD_DEACTV 0x2B
#define REG_B_EMD_SUP_CONF      0x05   /* space B */
#define REG_B_CORR_CONF1        0x0C   /* space B */
#define REG_B_CORR_CONF2        0x0D   /* space B */
#define REG_B_AUX_MOD           0x28   /* space B */
#define REG_B_PT_MOD            0x29   /* space B */
#define REG_B_RES_AM_MOD        0x2A   /* space B */
#define REG_B_OVERSHOOT_CONF1   0x30   /* space B */
#define REG_B_OVERSHOOT_CONF2   0x31   /* space B */
#define REG_B_UNDERSHOOT_CONF1  0x32   /* space B */
#define REG_B_UNDERSHOOT_CONF2  0x33   /* space B */
#define REG_REGULATOR_CONTROL   0x2C

/* ---- Regulator control register (0x2C) ---- */
#define REGULATOR_CONTROL_REG_S (1u << 7)

/* ---- IO configuration register 2 (0x01, Table 20): sup3V (this board's
 * DLDO1 rail runs the chip at 3.3V - without this bit the internal
 * regulator/reference voltages are mistuned for a 5V part running at
 * 3.3V), the MISO pulldowns and AAT-enable from apply_analog_defaults()'s
 * chip-init group folded into one absolute write (this register's other
 * bits - single/rfo2/i2c_thd/io_drv_lvl/slow_up - are fine left at their
 * post-Set-default reset value of 0). ---- */
#define IOCONF2_INIT_VALUE 0xB8   /* sup3V | miso_pd1 | miso_pd2 | aat_en */

/* ---- Operation control register (0x02, Table 21) ---- */
#define OP_EN     (1u << 7)
#define OP_RX_EN  (1u << 6)
#define OP_TX_EN  (1u << 3)
#define OP_EN_FD_MASK        0x03   /* en_fd_c<1:0> */
#define OP_EN_FD_MANUAL_CA   0x01   /* manual EFD, collision-avoidance threshold - required while using NFC field ON commands */
#define OP_EN_FD_AUTO        0x03   /* automatic - this driver's steady-state default, matching RFAL's rfalInitialize() */

/* ---- Mode definition register (0x03, Table 22/23): ISO14443A initiator
 * (reader) mode - targ=0 (initiator), om<3:0>=0001. ---- */
#define MODE_ISO14443A_INITIATOR 0x08

/* ---- Bit rate definition register (0x04, Table 25/26): 106 kbit/s both
 * directions (rate<1:0> = 00 for both tx_rate and rx_rate). ---- */
#define BITRATE_106K 0x00

/* ---- ISO14443A and NFC 106kb/s settings register (0x05, Table 27) ---- */
#define ISO14443A_ANTCL (1u << 0)   /* bit-oriented anticollision framing, reader mode only */

/* ---- Main interrupt register (0x1A, Table 62) ---- */
#define IRQ_MAIN_TXE (1u << 3)   /* end of transmission */
#define IRQ_MAIN_RXE (1u << 4)   /* end of receive */

/* ---- Timer and NFC interrupt register (0x1B, Table 63) ---- */
#define REG_TIMER_NFC_IRQ 0x1B
#define IRQ_TIMER_CAC (1u << 2)   /* collision during RF collision avoidance */
#define IRQ_TIMER_CAT (1u << 1)   /* minimum guard time expired, no collision */

/* ---- Passive target interrupt register (0x1D, Table 65) ---- */
#define REG_PASSIVE_TARGET_IRQ 0x1D
#define IRQ_PT_APON (1u << 5)   /* RF collision avoidance done, field switched on */

/* ---- Auxiliary display register (0x31) - NOT in the primary datasheet
 * text extracted for this driver's original bring-up; address and osc_ok
 * bit position cross-checked against ST's own reference driver
 * (stm32duino/ST25R3916, st25r3916_com.h) after the Main-interrupt-register
 * I_osc bit alone proved unreliable on real hardware. ST's own st25r3916OscOn()
 * treats THIS register as the definitive "oscillator actually stable"
 * signal, using the Main interrupt register's I_osc bit only as a wake-up,
 * not as ground truth - this driver now does the same. ---- */
#define AUX_DISPLAY_OSC_OK (1u << 4)

/* ---- IC identity register (0x3F, read-only) - chip type/revision. Cross-
 * checked against the same reference driver: bits 7:3 = ic_type, expected
 * 0b00101 (5) for a genuine ST25R3916. The only reliable "is this chip
 * really there" signal available (no separate GetStatus-style command). */
#define IC_IDENTITY_TYPE_MASK 0xF8
#define IC_IDENTITY_TYPE_ST25R3916 0x28

/* ---- direct commands (Table 13). These hex values already encode the SPI
 * mode bits (11) in their top two bits (e.g. 0xC0 = 0b11_000000), so
 * sending one raw byte is the entire command - no address/data phase. ---- */
#define DCMD_SET_DEFAULT     0xC0
#define DCMD_STOP_ALL        0xC2
#define DCMD_TX_WITH_CRC     0xC4
#define DCMD_TX_WITHOUT_CRC  0xC5
#define DCMD_TX_REQA         0xC6
#define DCMD_NFC_INITIAL_FIELD_ON 0xC8
#define DCMD_ADJUST_REGULATORS 0xD6
#define DCMD_MEASURE_AMPLITUDE 0xD3
#define DCMD_RESET_RXGAIN    0xD5
#define DCMD_CLEAR_FIFO      0xDB
#define REG_AD_CONVERTER_OUTPUT 0x25

/* ---- SPI mode-byte prefixes (Table 11) ---- */
#define SPI_REG_READ_BIT  0x40   /* OR into the 6-bit address for a register read */
#define SPI_FIFO_LOAD     0x80
#define SPI_FIFO_READ     0x9F

#define POLL_STEP_MS       2
#define OSC_START_TIMEOUT_MS 500
#define FIELD_ON_TIMEOUT_MS  30   /* reference driver's own budget is 10ms; small margin */
#define ADJUST_REGULATORS_TIMEOUT_MS 10   /* datasheet: max 5ms */

static spi_device_handle_t s_spi;
static i2c_master_dev_handle_t s_pmu;

/* ---- raw SPI primitives ---- */

static esp_err_t reg_write(uint8_t addr, const uint8_t *data, size_t len)
{
    uint8_t tx[1 + 8];
    if (len > sizeof(tx) - 1) {
        return ESP_ERR_INVALID_ARG;
    }
    tx[0] = addr & 0x3F;   /* top two bits 00 = register write */
    memcpy(tx + 1, data, len);
    spi_transaction_t t = {
        .length = (size_t)(1 + len) * 8,
        .tx_buffer = tx,
    };
    return spi_device_polling_transmit(s_spi, &t);
}

static esp_err_t reg_write1(uint8_t addr, uint8_t data)
{
    return reg_write(addr, &data, 1);
}

static esp_err_t reg_read(uint8_t addr, uint8_t *data, size_t len)
{
    uint8_t tx[1 + 8] = { 0 };
    uint8_t rx[1 + 8] = { 0 };
    if (len > sizeof(tx) - 1) {
        return ESP_ERR_INVALID_ARG;
    }
    tx[0] = (addr & 0x3F) | SPI_REG_READ_BIT;   /* top two bits 01 = register read */
    spi_transaction_t t = {
        .length = (size_t)(1 + len) * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    esp_err_t err = spi_device_polling_transmit(s_spi, &t);
    if (err == ESP_OK) {
        memcpy(data, rx + 1, len);
    }
    return err;
}

static esp_err_t reg_read1(uint8_t addr, uint8_t *data)
{
    return reg_read(addr, data, 1);
}

static esp_err_t reg_change_bits(uint8_t addr, uint8_t mask, uint8_t value)
{
    uint8_t cur = 0;
    ESP_RETURN_ON_ERROR(reg_read1(addr, &cur), TAG, "rmw read 0x%02x", addr);
    cur = (uint8_t)((cur & ~mask) | (value & mask));
    return reg_write1(addr, cur);
}

/* Space-B registers (Table 20's aat_en etc. share the "normal" register
 * write/read framing, just prefixed with the 0xFB space-B-select byte in
 * the SAME spi_device_polling_transmit() call - "access to register space-B
 * remains active until the rising edge of BSS" (datasheet SS4.3.3), i.e.
 * for exactly one continuous CS-low transaction, not two separate ones. */
static esp_err_t reg_write_b(uint8_t addr, const uint8_t *data, size_t len)
{
    uint8_t tx[2 + 8];
    if (len > sizeof(tx) - 2) {
        return ESP_ERR_INVALID_ARG;
    }
    tx[0] = 0xFB;
    tx[1] = addr & 0x3F;
    memcpy(tx + 2, data, len);
    spi_transaction_t t = {
        .length = (size_t)(2 + len) * 8,
        .tx_buffer = tx,
    };
    return spi_device_polling_transmit(s_spi, &t);
}

static esp_err_t reg_write1_b(uint8_t addr, uint8_t data)
{
    return reg_write_b(addr, &data, 1);
}

static esp_err_t reg_read1_b(uint8_t addr, uint8_t *data)
{
    uint8_t tx[3] = { 0xFB, (uint8_t)((addr & 0x3F) | SPI_REG_READ_BIT), 0 };
    uint8_t rx[3] = { 0 };
    spi_transaction_t t = {
        .length = sizeof(tx) * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    esp_err_t err = spi_device_polling_transmit(s_spi, &t);
    if (err == ESP_OK) {
        *data = rx[2];
    }
    return err;
}

static esp_err_t reg_change_bits_b(uint8_t addr, uint8_t mask, uint8_t value)
{
    uint8_t cur = 0;
    ESP_RETURN_ON_ERROR(reg_read1_b(addr, &cur), TAG, "rmw(B) read 0x%02x", addr);
    cur = (uint8_t)((cur & ~mask) | (value & mask));
    return reg_write1_b(addr, cur);
}

static esp_err_t direct_cmd(uint8_t code)
{
    uint8_t tx[1] = { code };
    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = tx,
    };
    return spi_device_polling_transmit(s_spi, &t);
}

/* FIFO load/read use their own mode byte (Table 11), not a register
 * address - up to 16 bytes of payload follow, plenty for REQA/ATQA and the
 * anticollision/select cascade frames this driver ever sends or reads. */
static esp_err_t fifo_load(const uint8_t *data, size_t len)
{
    uint8_t tx[1 + 16];
    if (len > sizeof(tx) - 1) {
        return ESP_ERR_INVALID_ARG;
    }
    tx[0] = SPI_FIFO_LOAD;
    memcpy(tx + 1, data, len);
    spi_transaction_t t = {
        .length = (size_t)(1 + len) * 8,
        .tx_buffer = tx,
    };
    return spi_device_polling_transmit(s_spi, &t);
}

static esp_err_t fifo_read(uint8_t *buf, size_t len)
{
    uint8_t tx[1 + 16] = { 0 };
    uint8_t rx[1 + 16] = { 0 };
    if (len > sizeof(tx) - 1) {
        return ESP_ERR_INVALID_ARG;
    }
    tx[0] = SPI_FIFO_READ;
    spi_transaction_t t = {
        .length = (size_t)(1 + len) * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    esp_err_t err = spi_device_polling_transmit(s_spi, &t);
    if (err == ESP_OK) {
        memcpy(buf, rx + 1, len);
    }
    return err;
}

static esp_err_t fifo_byte_count(size_t *n)
{
    uint8_t s1 = 0, s2 = 0;
    ESP_RETURN_ON_ERROR(reg_read1(REG_FIFO_STATUS1, &s1), TAG, "fifo status1");
    ESP_RETURN_ON_ERROR(reg_read1(REG_FIFO_STATUS2, &s2), TAG, "fifo status2");
    *n = ((size_t)(s2 & 0x03) << 8) | s1;   /* fifo_b9:8 in status2 bits7:6, fifo_b7:0 in status1 */
    return ESP_OK;
}

/* Host-clock-bounded wait for a Main-interrupt bit - the equivalent of
 * sx1262_recv()'s host-bounded poll loop, but there's no BUSY-style pin
 * here: this chip signals completion in the Main interrupt register
 * instead, and reading that register clears it (datasheet SS4.3.1), so
 * this is a single-shot wait for one bit per call, not a level check. */
static esp_err_t wait_reg_bit(uint8_t addr, uint8_t bit, TickType_t deadline)
{
    uint8_t last = 0;
    for (;;) {
        uint8_t val = 0;
        esp_err_t err = reg_read1(addr, &val);
        if (err != ESP_OK) {
            return err;
        }
        last = val;
        if (val & bit) {
            return ESP_OK;
        }
        if (xTaskGetTickCount() >= deadline) {
            ESP_LOGW(TAG, "wait_reg_bit(reg=0x%02x, bit=0x%02x) timed out, last read = 0x%02x",
                     addr, bit, last);
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(pdMS_TO_TICKS(POLL_STEP_MS));
    }
}

/* Wait for a Main-interrupt bit specifically - reading that register clears
 * it (datasheet SS4.3.1), so this is a single-shot wait for one bit per
 * call, not a level check. Used for RX-done during REQA/anticollision;
 * oscillator-stable is checked via the Auxiliary display register instead
 * (see wait_reg_bit(REG_AUX_DISPLAY, ...) in st25r3916_poll()) - the Main
 * interrupt register's I_osc bit proved unreliable on real hardware. */
static esp_err_t wait_main_irq(uint8_t bit, TickType_t deadline)
{
    return wait_reg_bit(REG_MAIN_IRQ, bit, deadline);
}

/* ---- ISO14443-A anticollision/select cascade (SS6.4 of the ISO14443-A
 * spec, not St's datasheet - this state machine has no chip assistance
 * beyond antcl/nbtx framing and is the same regardless of which cascade
 * level it's running at, just with a different SEL command byte. ---- */

static const uint8_t s_sel_cmd[3] = { 0x93, 0x95, 0x97 };   /* SEL_CL1/2/3 */

/* One cascade level: anticollision (NVB=0x20, no CRC, antcl framing) to get
 * 4 UID bytes + BCC, then select (NVB=0x70, with CRC) to get a 1-byte SAK.
 * uid4 always receives exactly 4 bytes - the caller decides whether byte 0
 * being 0x88 (cascade tag marker) means another level is needed. */
static esp_err_t cascade_level(uint8_t level, uint8_t *uid4, uint8_t *sak, TickType_t deadline)
{
    uint8_t sel = s_sel_cmd[level];

    /* Anticollision request: SEL + NVB(0x20) = "give me your whole UID",
     * the simplest single-tag case (no partial-UID bits already known). */
    ESP_RETURN_ON_ERROR(reg_write1(REG_ISO14443A_NFC, ISO14443A_ANTCL), TAG, "antcl on");
    ESP_RETURN_ON_ERROR(direct_cmd(DCMD_CLEAR_FIFO), TAG, "clear fifo (anticoll)");
    uint8_t anticoll[2] = { sel, 0x20 };
    ESP_RETURN_ON_ERROR(fifo_load(anticoll, sizeof(anticoll)), TAG, "fifo load anticoll");
    /* Number of transmitted bytes registers (Table 70/71): 2 full bytes,
     * 0 extra bits - the anticollision *request* is a whole-byte frame,
     * only a colliding *reply* would need bit-level handling, which this
     * single-tag milestone doesn't attempt to resolve. */
    ESP_RETURN_ON_ERROR(reg_write1(REG_NUM_TX_BYTES1, 0x00), TAG, "ntx1 (anticoll)");
    ESP_RETURN_ON_ERROR(reg_write1(REG_NUM_TX_BYTES2, (uint8_t)(2u << 3)), TAG, "ntx2 (anticoll)");
    ESP_RETURN_ON_ERROR(direct_cmd(DCMD_TX_WITHOUT_CRC), TAG, "tx anticoll");
    esp_err_t err = wait_main_irq(IRQ_MAIN_RXE, deadline);
    if (err != ESP_OK) {
        return err;
    }
    size_t n = 0;
    ESP_RETURN_ON_ERROR(fifo_byte_count(&n), TAG, "fifo count (anticoll)");
    if (n < 5) {
        return ESP_ERR_INVALID_RESPONSE;   /* need 4 UID bytes + BCC */
    }
    uint8_t reply[5];
    ESP_RETURN_ON_ERROR(fifo_read(reply, sizeof(reply)), TAG, "fifo read anticoll");
    memcpy(uid4, reply, 4);

    /* Select: SEL + NVB(0x70) + the 4 UID bytes + BCC just read back -
     * a complete (not anticollision) frame, so CRC is used and antcl framing
     * is turned back off. Response is SAK (1 byte, CRC stripped/checked by
     * the chip - a CRC mismatch isn't separately verified here, matching
     * this milestone's "trust it" scope). */
    ESP_RETURN_ON_ERROR(reg_write1(REG_ISO14443A_NFC, 0x00), TAG, "antcl off");
    ESP_RETURN_ON_ERROR(direct_cmd(DCMD_CLEAR_FIFO), TAG, "clear fifo (select)");
    uint8_t select[7] = { sel, 0x70, reply[0], reply[1], reply[2], reply[3], reply[4] };
    ESP_RETURN_ON_ERROR(fifo_load(select, sizeof(select)), TAG, "fifo load select");
    ESP_RETURN_ON_ERROR(reg_write1(REG_NUM_TX_BYTES1, 0x00), TAG, "ntx1 (select)");
    ESP_RETURN_ON_ERROR(reg_write1(REG_NUM_TX_BYTES2, (uint8_t)(7u << 3)), TAG, "ntx2 (select)");
    ESP_RETURN_ON_ERROR(direct_cmd(DCMD_TX_WITH_CRC), TAG, "tx select");
    err = wait_main_irq(IRQ_MAIN_RXE, deadline);
    if (err != ESP_OK) {
        return err;
    }
    ESP_RETURN_ON_ERROR(fifo_byte_count(&n), TAG, "fifo count (select)");
    if (n < 1) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    return fifo_read(sak, 1);
}

/* Board-level analog/RF tuning (TX driver, receiver gain/AGC, correlator,
 * antenna trim, field-detector thresholds, overshoot/undershoot protection)
 * that the datasheet alone doesn't specify a value for - see this file's
 * top-of-file comment for where these values come from. Narrowed to the
 * final register state needed for NFC-A/106kbit/OOK reader operation (the
 * only mode this driver ever uses), skipping RFAL's AM-modulation and
 * other-bitrate entries that would just get overridden back to these same
 * values whenever this driver's use case is active. Split into two phases
 * because some registers (IO_CONF1/2, TX_DRIVER, field thresholds, PT_MOD,
 * antenna trim, ...) must be set before the oscillator/Rx/Tx are enabled,
 * while others (receiver gain/correlator/over-undershoot) only matter once
 * the chip is actually about to transmit/receive. */
static esp_err_t apply_analog_defaults_pre_osc(void)
{
    esp_err_t err = ESP_OK;
    err |= reg_change_bits(REG_IO_CONF1, 0x07, 0x07);                 /* disable MCU_CLK */
    err |= reg_write1(REG_IO_CONF2, IOCONF2_INIT_VALUE);              /* sup3V, MISO pulldowns, AAT enable */
    err |= reg_change_bits(REG_TX_DRIVER, 0x0F, 0x00);                /* RFO resistance, active TX */
    err |= reg_write1_b(REG_B_RES_AM_MOD, 0x80);                      /* minimum non-overlap */
    err |= reg_change_bits(REG_FIELD_THRESHOLD_ACTV, 0x7F, 0x13);     /* activation threshold */
    err |= reg_change_bits(REG_FIELD_THRESHOLD_DEACTV, 0x7F, 0x02);   /* deactivation threshold */
    err |= reg_change_bits_b(REG_B_AUX_MOD, 0x30, 0x10);              /* internal (not external) load modulation */
    err |= reg_change_bits(REG_PASSIVE_TARGET, 0xF0, 0x50);           /* FDT alignment with the bit grid */
    err |= reg_write1_b(REG_B_PT_MOD, 0x5F);                          /* reduced RFO resistance, modulated state */
    err |= reg_change_bits_b(REG_B_EMD_SUP_CONF, 0x40, 0x40);         /* EMD suppression: start on first 4 bits */
    err |= reg_write1(REG_ANT_TUNE_A, 0x82);                          /* antenna tuning (poller): ANTL */
    err |= reg_write1(REG_ANT_TUNE_B, 0x82);
    return (err == ESP_OK) ? ESP_OK : ESP_FAIL;
}

static esp_err_t apply_analog_defaults_nfca_106(void)
{
    esp_err_t err = ESP_OK;
    err |= reg_change_bits(REG_AUX, 0x04, 0x00);            /* correlator receiver, not coherent */
    err |= reg_write1_b(REG_B_OVERSHOOT_CONF1, 0x40);
    err |= reg_write1_b(REG_B_OVERSHOOT_CONF2, 0x03);
    err |= reg_write1_b(REG_B_UNDERSHOOT_CONF1, 0x40);
    err |= reg_write1_b(REG_B_UNDERSHOOT_CONF2, 0x03);
    err |= reg_write1(REG_RX_CONF1, 0x08);
    err |= reg_write1(REG_RX_CONF2, 0x2D);
    err |= reg_write1(REG_RX_CONF3, 0x00);
    err |= reg_write1(REG_RX_CONF4, 0x00);
    /* CORR_CONF1 = 0x11: the NFC-A/106k base value (0x51) with corr_s6
     * cleared, folding in RFAL's separate "anticollision" adjustment (see
     * this file's top-of-file comment on why this driver doesn't
     * dynamically switch configs per frame type like RFAL does). */
    err |= reg_write1_b(REG_B_CORR_CONF1, 0x11);
    err |= reg_write1_b(REG_B_CORR_CONF2, 0x00);
    return (err == ESP_OK) ? ESP_OK : ESP_FAIL;
}

/* One-time-per-bring-up regulator calibration (datasheet "Adjust
 * regulators" direct command, 0xD6) - RFAL's rfalCalibrate() runs this once
 * during rfalInitialize() and this driver had been skipping it entirely.
 * Toggling reg_s (Regulator control register bit7) resets the regulator
 * logic so the command recalculates from scratch rather than reusing a
 * stale value; the actual resulting voltage isn't read back here since
 * this driver has no use for the number, only the calibration side effect. */
static esp_err_t calibrate_regulators(void)
{
    ESP_RETURN_ON_ERROR(reg_change_bits(REG_REGULATOR_CONTROL, REGULATOR_CONTROL_REG_S, REGULATOR_CONTROL_REG_S),
                         TAG, "regulator reset (set)");
    ESP_RETURN_ON_ERROR(reg_change_bits(REG_REGULATOR_CONTROL, REGULATOR_CONTROL_REG_S, 0x00),
                         TAG, "regulator reset (clear)");
    ESP_RETURN_ON_ERROR(direct_cmd(DCMD_ADJUST_REGULATORS), TAG, "adjust regulators");
    vTaskDelay(pdMS_TO_TICKS(ADJUST_REGULATORS_TIMEOUT_MS));
    return ESP_OK;
}

/* Turn the RF field on via the "NFC initial field ON" direct command
 * (0xC8) - the datasheet/RFAL mechanism. A direct tx_en write was tried
 * first and measurably does NOT work: an RFI-amplitude diagnostic read 0
 * with tx_en poked directly (field off in practice, despite the bit
 * reading back set), vs. a consistent ~15-16/255 with this C8 path (field
 * genuinely on) - confirms the datasheet's own implication that tx_en
 * "is automatically set by NFC Field ON commands" and isn't meant to be
 * written directly.
 *
 * The expected completion signal (I_apon or I_cac, Passive target/Timer-NFC
 * interrupt registers) has never been observed to fire on this hardware,
 * even though the field demonstrably does turn on regardless - this
 * driver's IRQ-based completion check itself has an unresolved bug (wrong
 * bit, wrong register, or a read-timing issue), so it's treated as
 * non-fatal: log it and continue, since the field is on either way.
 *
 * With the field confirmed on (amplitude test) and REQA confirmed
 * transmitting cleanly, a tag confirmed working under stock firmware still
 * never answers - the ~15-16/255 amplitude reading is low, so the leading
 * theory is field strength (this driver's antenna trim registers are
 * RFAL's generic defaults, not calibrated for this specific board's
 * antenna), not a further protocol bug, but this hasn't been confirmed. */
static esp_err_t field_on(void)
{
    ESP_RETURN_ON_ERROR(reg_change_bits(REG_OP_CONTROL, OP_EN_FD_MASK, OP_EN_FD_MANUAL_CA),
                         TAG, "en_fd manual/ca");
    ESP_RETURN_ON_ERROR(direct_cmd(DCMD_NFC_INITIAL_FIELD_ON), TAG, "nfc initial field on");

    esp_err_t err = ESP_ERR_TIMEOUT;
    TickType_t ca_deadline = xTaskGetTickCount() + pdMS_TO_TICKS(FIELD_ON_TIMEOUT_MS);
    for (;;) {
        uint8_t pt_irq = 0, timer_irq = 0;
        reg_read1(REG_PASSIVE_TARGET_IRQ, &pt_irq);
        reg_read1(REG_TIMER_NFC_IRQ, &timer_irq);
        if (timer_irq & IRQ_TIMER_CAC) {
            ESP_LOGW(TAG, "field_on: RF collision detected during collision avoidance");
            err = ESP_ERR_INVALID_STATE;
            break;
        }
        if (pt_irq & IRQ_PT_APON) {
            err = wait_reg_bit(REG_TIMER_NFC_IRQ, IRQ_TIMER_CAT,
                                xTaskGetTickCount() + pdMS_TO_TICKS(FIELD_ON_TIMEOUT_MS));
            break;
        }
        if (xTaskGetTickCount() >= ca_deadline) {
            /* Non-fatal: the field turns on regardless of this check ever
             * completing (verified via RFI amplitude), so don't block the
             * caller on a completion signal this driver can't currently see. */
            err = ESP_OK;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(POLL_STEP_MS));
    }

    reg_change_bits(REG_OP_CONTROL, OP_EN_FD_MASK, OP_EN_FD_AUTO);
    return err;
}

/* ---- public API ---- */

esp_err_t st25r3916_init(spi_device_handle_t spi, i2c_master_dev_handle_t pmu)
{
    s_spi = spi;
    s_pmu = pmu;

    gpio_config_t irq_io = {
        .pin_bit_mask = 1ULL << ST25R3916_PIN_IRQ,
        .mode = GPIO_MODE_INPUT,
    };
    gpio_config(&irq_io);

    /* DLDO1 stays off until a poll actually happens (st25r3916_poll() owns
     * it on demand) - nothing further to configure or probe here without
     * powering the rail just to immediately power it back off again. */
    ESP_LOGI(TAG, "st25r3916 registered (CS/IRQ wired, DLDO1 powered on demand by poll)");
    return ESP_OK;
}

esp_err_t st25r3916_open(void)
{
    axp2101_enable_rail(s_pmu, AXP2101_DLDO1, true);
    vTaskDelay(pdMS_TO_TICKS(5));   /* chip power-up settle before the first SPI transaction */

    /* Set default (0xC0) is accepted in any state - lands the chip in a
     * known power-up configuration regardless of what a previous session
     * left behind. */
    esp_err_t err = direct_cmd(DCMD_SET_DEFAULT);

    /* Chip-presence check via the IC identity register - the only reliable
     * "is this chip really there" signal available (no GetStatus-style
     * command on this chip). A mismatch means the chip isn't actually
     * answering SPI (absent, unpowered, wrong SPI mode, ...) - hard-fail
     * here rather than continue into a bring-up sequence that would just
     * read back more garbage. */
    uint8_t ic_id = 0;
    if (err == ESP_OK) {
        err = reg_read1(REG_IC_IDENTITY, &ic_id);
    }
    if (err != ESP_OK || (ic_id & IC_IDENTITY_TYPE_MASK) != IC_IDENTITY_TYPE_ST25R3916) {
        ESP_LOGW(TAG, "st25r3916 not responding correctly (ic_identity=0x%02x, expected type 0x%02x in mask 0x%02x) - chip absent/unpowered/misconfigured",
                 ic_id, IC_IDENTITY_TYPE_ST25R3916, IC_IDENTITY_TYPE_MASK);
        axp2101_enable_rail(s_pmu, AXP2101_DLDO1, false);
        return (err == ESP_OK) ? ESP_ERR_INVALID_RESPONSE : err;
    }

    if (err == ESP_OK) {
        err = apply_analog_defaults_pre_osc();
    }
    /* Enable the oscillator/regulator alone first (not Rx/Tx yet) and wait
     * for it to actually stabilize before touching anything else - Mode/
     * Bitrate are documented as "can be written only in case crystal clock
     * is present and stable". Ground truth is the Auxiliary display
     * register's osc_ok bit, not the Main interrupt register's I_osc bit -
     * see wait_reg_bit()'s doc comment for why. This mirrors ST's own
     * reference driver's st25r3916OscOn() (stm32duino/ST25R3916), which
     * enables 'en' alone here and only sets rx_en/tx_en afterward. */
    if (err == ESP_OK) {
        err = reg_write1(REG_OP_CONTROL, OP_EN);
    }
    if (err == ESP_OK) {
        err = wait_reg_bit(REG_AUX_DISPLAY, AUX_DISPLAY_OSC_OK,
                            xTaskGetTickCount() + pdMS_TO_TICKS(OSC_START_TIMEOUT_MS));
    }
    if (err == ESP_OK) {
        err = calibrate_regulators();
    }
    if (err == ESP_OK) {
        err = reg_write1(REG_OP_CONTROL, OP_EN | OP_RX_EN);   /* tx_en NOT set here - see field_on() */
    }
    if (err == ESP_OK) {
        err = reg_write1(REG_MODE, MODE_ISO14443A_INITIATOR);
    }
    if (err == ESP_OK) {
        err = reg_write1(REG_BITRATE, BITRATE_106K);
    }
    if (err == ESP_OK) {
        err = apply_analog_defaults_nfca_106();
    }
    if (err == ESP_OK) {
        err = field_on();
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "st25r3916 bring-up failed: %s", esp_err_to_name(err));
        axp2101_enable_rail(s_pmu, AXP2101_DLDO1, false);
        return err;
    }

    /* Diagnostic-only: measure the RFI input amplitude with the field on,
     * independent of REQA/ATQA framing - separates "TX isn't radiating
     * anything" from "TX radiates fine, something's wrong receiving/
     * decoding a response". Not gated on any completion IRQ - "Measure
     * amplitude" is a plain, fast (<1ms) A/D conversion. */
    direct_cmd(DCMD_MEASURE_AMPLITUDE);
    vTaskDelay(pdMS_TO_TICKS(2));
    uint8_t amplitude = 0;
    if (reg_read1(REG_AD_CONVERTER_OUTPUT, &amplitude) == ESP_OK) {
        ESP_LOGI(TAG, "RFI amplitude reading: %u (0-255, higher = more signal)", amplitude);
    }
    uint8_t op_after_field_on = 0;
    reg_read1(REG_OP_CONTROL, &op_after_field_on);
    ESP_LOGI(TAG, "op_control after field_on(): 0x%02x (tx_en bit is 0x%02x)", op_after_field_on, OP_TX_EN);

    return ESP_OK;
}

void st25r3916_close(void)
{
    axp2101_enable_rail(s_pmu, AXP2101_DLDO1, false);
}

esp_err_t st25r3916_try(st25r3916_tag_t *tag, int timeout_ms)
{
    if (!tag) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(tag, 0, sizeof(*tag));

    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);

    /* RFAL's rfalPrepareTransceive() (reader/writer branch) resets the
     * receive logic with "Stop all activities" + "Reset RX gain" before
     * every transceive. Tried here, but on this hardware STOP_ALL clears
     * en/rx_en/tx_en wholesale (op_control read back 0x03 - only en_fd_c
     * survived, not just tx_en as the datasheet's wording alone would
     * suggest), and restoring from that mid-session proved unreliable
     * (the oscillator wait after re-enabling 'en' intermittently timed out
     * too) - net negative versus this driver's simpler Clear-FIFO-only
     * reset, which was already working reliably for REQA transmission
     * itself. Left out; only the "wait for TXE before RXE" fix below (a
     * separate, uncontroversial RFAL behavior unrelated to STOP_ALL) is
     * kept. */
    direct_cmd(DCMD_CLEAR_FIFO);
    reg_write1(REG_ISO14443A_NFC, 0x00);   /* antcl off - REQA/WUPA get automatic CRC-free handling */
    /* "Clear nbtx bits before sending WUPA/REQA - otherwise ST25R3916 will
     * report parity error" (RFAL's own comment at this exact point in
     * rfalISO14443ATransceiveShortFrame()). Stale nbtx bits shouldn't
     * persist here in practice (nothing else sets them before this point in
     * a session), but it's what the reference sequence does. */
    reg_write1(REG_NUM_TX_BYTES2, 0x00);

    esp_err_t err = direct_cmd(DCMD_TX_REQA);
    /* Wait for the transmission to actually finish (I_txe) before waiting
     * for a response - this driver's first version raced straight to
     * waiting for I_rxe right after issuing the command, without confirming
     * TX had completed. RFAL's rfalISO14443ATransceiveShortFrame() always
     * waits for I_txe first, only entering its RX-wait state afterward. */
    if (err == ESP_OK) {
        err = wait_main_irq(IRQ_MAIN_TXE, deadline);
    }
    if (err == ESP_OK) {
        err = wait_main_irq(IRQ_MAIN_RXE, deadline);
    }
    if (err != ESP_OK) {
        /* No tag in range - not a driver error, just nothing to report. */
        return (err == ESP_ERR_TIMEOUT) ? ESP_ERR_NOT_FOUND : err;
    }
    size_t n = 0;
    fifo_byte_count(&n);
    uint8_t atqa[2] = { 0 };
    if (n >= 2) {
        fifo_read(atqa, 2);
    }

    uint8_t uid[10] = { 0 };
    uint8_t uid_len = 0;
    for (uint8_t level = 0; level < 3; level++) {
        uint8_t part[4] = { 0 }, sak = 0;
        err = cascade_level(level, part, &sak, deadline);
        if (err != ESP_OK) {
            return err;
        }
        bool cascade_more = (part[0] == 0x88);   /* cascade tag marker */
        size_t copy_n = cascade_more ? 3 : 4;
        size_t copy_off = cascade_more ? 1 : 0;
        if ((size_t)uid_len + copy_n > sizeof(uid)) {
            return ESP_ERR_INVALID_RESPONSE;
        }
        memcpy(uid + uid_len, part + copy_off, copy_n);
        uid_len = (uint8_t)(uid_len + copy_n);
        if (!cascade_more) {
            break;
        }
    }

    memcpy(tag->uid, uid, uid_len);
    tag->uid_len = uid_len;
    tag->found = true;

    char hex[3 * 10 + 1] = { 0 };
    for (uint8_t i = 0; i < uid_len; i++) {
        snprintf(hex + i * 3, 4, "%02X ", uid[i]);
    }
    ESP_LOGI(TAG, "tag found: ATQA=%02x%02x UID=%s(%u bytes)", atqa[0], atqa[1], hex, uid_len);
    return ESP_OK;
}

esp_err_t st25r3916_poll(st25r3916_tag_t *tag, int timeout_ms)
{
    esp_err_t err = st25r3916_open();
    if (err != ESP_OK) {
        st25r3916_close();
        return err;
    }
    err = st25r3916_try(tag, timeout_ms);
    st25r3916_close();
    return err;
}
