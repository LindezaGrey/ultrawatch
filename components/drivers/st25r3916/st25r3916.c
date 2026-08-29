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
#include "spi2_power.h"
#include "esp_check.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <inttypes.h>
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
/* 01 is what reader mode wants and what it keeps: datasheet SS4.4.5 Note -
 * "It is recommended to set to 01 bits en_fd_c<1:0> of Operation control
 * register in Reader mode"; 10/11 are for NFCIP-1 active communication (AP2P)
 * and passive target. This used to be flipped to 11 ("automatic") once
 * field-on completed, which is wrong twice over: it is the AP2P setting, and
 * Table 21 notes that en_fd_c != 0 with every other op_control bit clear puts
 * the device into Low power initial NFC mode. */
#define OP_EN_FD_MANUAL_CA   0x01   /* manual EFD, collision-avoidance threshold - required while using NFC field ON commands */

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

/* ---- Error and wake-up interrupt register (0x1C, Table 64) ---- */
#define REG_ERROR_IRQ 0x1C
#define IRQ_ERR_CRC   (1u << 7)   /* CRC error */
#define IRQ_ERR_PAR   (1u << 6)   /* parity error */
#define IRQ_ERR_SOFT  (1u << 5)   /* soft framing error (Rx data still usable) */
#define IRQ_ERR_HARD  (1u << 4)   /* hard framing error (Rx data corrupted) */

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

/* SPI2 is shared by the SD card, the SX1262 and this chip. An unpowered
 * device on that bus does not release it: its I/O pins clamp the shared
 * MISO net through their own ESD protection diodes toward their dead 0V
 * supply, so EVERY read on SPI2 comes back 0x00 while any one of them is
 * off - not just reads of that device. Writes are unaffected (MOSI is
 * driven by the host), which is what made this so confusing the first
 * time around: the chip was being configured correctly the whole time and
 * simply could not be read. See spi2_power.h for the full mechanism and
 * docs/nfc.md for the hardware measurements that found it.
 *
 * st25r3916_open()/close() bracket every SPI2 session with
 * spi2_power_hold()/release(AXP2101_DLDO1), which keeps every other
 * hazard rail (the SD card's, LoRa's) up for the duration and also raises
 * this chip's own DLDO1 - registered as an SPI2_POWER_OWNED rail, which
 * spi2_power_release() never lowers on its own, matching this chip's
 * inability to reliably survive a rail power-cycle once it has completed
 * bring-up (see st25r3916_close()). */

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
    /* Table 67: fifo_b<9:8> are status2 bits 7:6. Bits 1:0 are fifo_lb0 and
     * np_lb (bits in the last byte / missing parity), so masking the low two
     * bits instead added 256 or 512 to the count on any frame whose last byte
     * was incomplete. */
    *n = ((size_t)((s2 >> 6) & 0x03) << 8) | s1;
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

/* ---- interrupt status: sticky snapshots, never single-shot reads ----
 *
 * The four status registers are contiguous (0x1A Main, 0x1B Timer/NFC, 0x1C
 * Error/wake-up, 0x1D Passive target), so one auto-incrementing read collects
 * all of them in a single transaction.
 *
 * Reading a status register resets its content to 0 (datasheet SS4.3.1), and
 * the same section notes that several bits can latch between two host reads.
 * A caller that waits for one bit with a plain read therefore *destroys*
 * every other event that arrived in the same window. That is not a corner
 * case here: a REQA -> ATQA exchange at 106kbit/s finishes in a few hundred
 * microseconds, far inside POLL_STEP_MS, so I_txe and I_rxe essentially
 * always arrive together. Waiting for I_txe and then, separately, for I_rxe
 * consumed I_rxe in the first wait and then blocked until timeout in the
 * second - which reads exactly like "no tag answered".
 *
 * So each read is accumulated into a sticky 32-bit mask owned by the caller
 * for the whole duration of a wait, and waits test the accumulator. Status
 * bits are also cleared by Set default, Stop all activities and Clear FIFO
 * (SS4.3.1), so a fresh mask must be started after those, not before. */
#define IRQ_MAIN(bits)   ((uint32_t)(uint8_t)(bits) << 0)
#define IRQ_TIMER(bits)  ((uint32_t)(uint8_t)(bits) << 8)
#define IRQ_ERROR(bits)  ((uint32_t)(uint8_t)(bits) << 16)
#define IRQ_TARGET(bits) ((uint32_t)(uint8_t)(bits) << 24)

#define IRQ_ERROR_ANY IRQ_ERROR(IRQ_ERR_CRC | IRQ_ERR_PAR | IRQ_ERR_SOFT | IRQ_ERR_HARD)

static esp_err_t irq_poll(uint32_t *sticky)
{
    uint8_t regs[4] = { 0 };
    ESP_RETURN_ON_ERROR(reg_read(REG_MAIN_IRQ, regs, sizeof(regs)), TAG, "irq snapshot");
    *sticky |= IRQ_MAIN(regs[0]) | IRQ_TIMER(regs[1]) |
               IRQ_ERROR(regs[2]) | IRQ_TARGET(regs[3]);
    return ESP_OK;
}

static esp_err_t wait_irq(uint32_t want, uint32_t *sticky, TickType_t deadline)
{
    for (;;) {
        ESP_RETURN_ON_ERROR(irq_poll(sticky), TAG, "irq poll");
        if (*sticky & want) {
            return ESP_OK;
        }
        if (xTaskGetTickCount() >= deadline) {
            ESP_LOGW(TAG, "wait_irq(want=0x%08" PRIx32 ") timed out, seen=0x%08" PRIx32,
                     want, *sticky);
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(pdMS_TO_TICKS(POLL_STEP_MS));
    }
}

/* Wait for a reply frame after a transmit command. Only I_rxe is waited on;
 * I_txe and the error bits are picked up into the same sticky mask along the
 * way, so a timeout can say whether the transmission itself even completed -
 * "TX never finished" and "TX fine, nothing answered" are very different
 * failures and used to be indistinguishable. */
static esp_err_t wait_response(uint32_t *sticky, TickType_t deadline)
{
    esp_err_t err = wait_irq(IRQ_MAIN(IRQ_MAIN_RXE), sticky, deadline);
    if (err == ESP_ERR_TIMEOUT) {
        ESP_LOGW(TAG, "no response: txe=%s, timer/nfc irq=0x%02x, error irq=0x%02x",
                 (*sticky & IRQ_MAIN(IRQ_MAIN_TXE)) ? "yes" : "NO",
                 (unsigned)((*sticky >> 8) & 0xFF), (unsigned)((*sticky >> 16) & 0xFF));
    } else if (err == ESP_OK && (*sticky & IRQ_ERROR_ANY)) {
        ESP_LOGW(TAG, "response had CRC/parity/framing errors (error irq=0x%02x)",
                 (unsigned)((*sticky >> 16) & 0xFF));
    }
    return err;
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
    /* Clear FIFO above already reset the interrupt status bits (SS4.3.1), so
     * this mask starts clean and stays valid for the whole exchange. */
    uint32_t irq = 0;
    ESP_RETURN_ON_ERROR(direct_cmd(DCMD_TX_WITHOUT_CRC), TAG, "tx anticoll");
    esp_err_t err = wait_response(&irq, deadline);
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
    irq = 0;
    ESP_RETURN_ON_ERROR(direct_cmd(DCMD_TX_WITH_CRC), TAG, "tx select");
    err = wait_response(&irq, deadline);
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
 * genuinely on) - confirms Table 21's own wording that tx_en "is
 * automatically set by NFC Field ON commands" and isn't meant to be written
 * directly.
 *
 * 0xC8 needs operation mode en (Table 13) and needs the external field
 * detector enabled (SS4.4.5), hence the en_fd_c write first. en_fd_c is then
 * left at 01 - reader mode's recommended setting - rather than being flipped
 * to 11 on the way out as it used to be.
 *
 * If no external field is present the transmitter switches on and I_apon is
 * signalled, followed by I_cat once the NFCIP-1 guard time has passed; an
 * external field instead gives I_cac and the transmitter stays off. */
static esp_err_t field_on(void)
{
    ESP_RETURN_ON_ERROR(reg_change_bits(REG_OP_CONTROL, OP_EN_FD_MASK, OP_EN_FD_MANUAL_CA),
                         TAG, "en_fd manual/ca");
    ESP_RETURN_ON_ERROR(direct_cmd(DCMD_NFC_INITIAL_FIELD_ON), TAG, "nfc initial field on");

    /* I_apon and I_cat are only ~a guard time apart and can easily land
     * between the same pair of polls, so both go into one sticky mask - the
     * previous version read 0x1D and 0x1B as two separate transactions per
     * iteration and threw away whichever bit it wasn't looking at, then
     * waited for an I_cat it had already consumed. */
    uint32_t irq = 0;
    esp_err_t err = wait_irq(IRQ_TARGET(IRQ_PT_APON) | IRQ_TIMER(IRQ_TIMER_CAC), &irq,
                             xTaskGetTickCount() + pdMS_TO_TICKS(FIELD_ON_TIMEOUT_MS));
    if (err == ESP_OK && (irq & IRQ_TIMER(IRQ_TIMER_CAC))) {
        ESP_LOGW(TAG, "field_on: external field detected during collision avoidance - transmitter stays off");
        return ESP_ERR_INVALID_STATE;
    }
    if (err == ESP_OK) {
        err = wait_irq(IRQ_TIMER(IRQ_TIMER_CAT), &irq,
                        xTaskGetTickCount() + pdMS_TO_TICKS(FIELD_ON_TIMEOUT_MS));
    }
    if (err != ESP_OK) {
        /* Non-fatal while the reader is still being brought up: the field
         * demonstrably comes on either way (RFI amplitude confirms it), and
         * the REQA that follows is a far more informative failure than
         * stopping here. */
        ESP_LOGW(TAG, "field_on: no I_apon/I_cat within %dms (irq seen 0x%08" PRIx32 ") - continuing",
                 FIELD_ON_TIMEOUT_MS, irq);
        err = ESP_OK;
    }
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
    /* What the PMU actually has programmed for DLDO1 (reg 0x99, low 5 bits
     * = (mV - 500) / 100). Read over I2C from the PMU, so it is valid
     * whether or not the NFC chip answers SPI - which makes it the number to
     * trust about this rail. (The chip's own "Measure power supply" command
     * is not: Table 13 lists it as requiring operation mode en, so any
     * reading taken before the oscillator is enabled is just whatever the
     * previous conversion left in register 0x25.) */
    uint8_t dldo1_vol = 0;
    if (axp2101_read_reg(s_pmu, 0x99, &dldo1_vol) == ESP_OK) {
        ESP_LOGI(TAG, "PMU DLDO1 vol reg 0x99 = 0x%02x -> %umV programmed",
                 dldo1_vol, (unsigned)(500 + (dldo1_vol & 0x1F) * 100));
    }

    /* Before anything is read back: raises every rail this session needs
     * to trust its SPI2 reads, including DLDO1 itself, and settles for
     * however long any of them actually needed (a no-op if everything was
     * already on - see spi2_power.h). */
    spi2_power_hold(AXP2101_DLDO1);

    /* Confirm the rail enable actually landed on DLDO1's bit (REG 0x90 bit7)
     * rather than being silently dropped - the enable bit was wrong until
     * just now, so this is worth verifying rather than assuming. */
    uint8_t onoff0 = 0;
    if (axp2101_read_reg(s_pmu, 0x90, &onoff0) == ESP_OK) {
        ESP_LOGI(TAG, "PMU LDO on/off reg 0x90 = 0x%02x (DLDO1=bit7 -> %s)",
                 onoff0, (onoff0 & (1u << 7)) ? "ON" : "OFF");
    }

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
        /* Bring-up never completed, so unlike a normal close() there is
         * nothing running to lose from a power cycle - actively lower
         * DLDO1 (bypassing spi2_power's SPI2_POWER_OWNED "never auto-lower"
         * policy on purpose, same as sd_log.c's own retry-toggle recovery
         * dance bypasses spi2_power for its own hardware-recovery reason)
         * so the next open() gets a genuine cold start rather than retrying
         * against a chip stuck in whatever state it's currently in. */
        axp2101_enable_rail(s_pmu, AXP2101_DLDO1, false);
        spi2_power_release(AXP2101_RAIL_MAX);
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
        /* See the identity-mismatch path above for why this bypasses
         * spi2_power's normal "never auto-lower DLDO1" policy. */
        axp2101_enable_rail(s_pmu, AXP2101_DLDO1, false);
        spi2_power_release(AXP2101_RAIL_MAX);
        return err;
    }

    /* Table 21: tx_en "is automatically set by NFC Field ON commands", so
     * op_control should read back en|rx_en|tx_en here with no help from us.
     *
     * It previously read back 0x03 - every enable clear, only en_fd_c set -
     * which was taken to mean 0xC8 wipes op_control, and was worked around by
     * force-writing the enables back. That theory does not survive the
     * datasheet: 0x03 is exactly what field_on()'s own closing
     * read-modify-write on en_fd_c produced when its *read* returned 0x00,
     * i.e. (0x00 & ~0x03) | 0x03. Marginal SPI reads (the device ran at 10MHz
     * against a 70ns max data-out delay) or a genuine power-on reset
     * (op_control resets to 0x00) both explain it; a chip quirk does not.
     * So: no force-write, just log what the chip actually reports. */
    uint8_t op_after_field_on = 0;
    reg_read1(REG_OP_CONTROL, &op_after_field_on);
    ESP_LOGI(TAG, "op_control after field_on(): 0x%02x (want en|rx_en|tx_en = 0x%02x)",
             op_after_field_on, (unsigned)(OP_EN | OP_RX_EN | OP_TX_EN));

    /* Diagnostic-only: RFI input amplitude with the field on, independent of
     * REQA/ATQA framing - separates "TX isn't radiating" from "TX radiates,
     * something's wrong receiving". "Measure amplitude" is a plain <1ms A/D
     * conversion (Table 13), so it needs no completion IRQ. */
    direct_cmd(DCMD_MEASURE_AMPLITUDE);
    vTaskDelay(pdMS_TO_TICKS(2));
    uint8_t amplitude = 0;
    if (reg_read1(REG_AD_CONVERTER_OUTPUT, &amplitude) == ESP_OK) {
        ESP_LOGI(TAG, "RFI amplitude reading: %u (0-255, higher = more signal)", amplitude);
    }

    return ESP_OK;
}

void st25r3916_close(void)
{
    /* spi2_power_release() leaves DLDO1 up (SPI2_POWER_OWNED - see the
     * file-top comment and spi2_power.h): the ST25R3916 has proven unable
     * to come back after a rail power-cycle once bring-up has completed,
     * and LilyGo's own firmware likewise enables DLDO1 once at boot and
     * leaves it on. Stopping the RF field is the useful part of "closing";
     * the SD/LoRa hazard rails are released normally, dropping if nothing
     * else needs them. */
    reg_change_bits(REG_OP_CONTROL, OP_TX_EN, 0x00);
    spi2_power_release(AXP2101_DLDO1);
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
     * every transceive. Not done here: Clear FIFO alone already resets the
     * FIFO, the FIFO status registers and the interrupt status bits
     * (datasheet SS4.3.1), which is what this driver needs, and STOP_ALL was
     * previously observed leaving op_control in a state this driver couldn't
     * recover from mid-session. That observation is now suspect for the same
     * reason as the field_on() one (see st25r3916_open()), so STOP_ALL is
     * worth re-testing once the read path is confirmed solid. */
    direct_cmd(DCMD_CLEAR_FIFO);
    reg_write1(REG_ISO14443A_NFC, 0x00);   /* antcl off - REQA/WUPA get automatic CRC-free handling */
    /* "Clear nbtx bits before sending WUPA/REQA - otherwise ST25R3916 will
     * report parity error" (RFAL's own comment at this exact point in
     * rfalISO14443ATransceiveShortFrame()). Stale nbtx bits shouldn't
     * persist here in practice (nothing else sets them before this point in
     * a session), but it's what the reference sequence does. */
    reg_write1(REG_NUM_TX_BYTES2, 0x00);

    /* Clear FIFO above cleared the interrupt status bits, so this mask starts
     * clean. One wait, not two: I_txe and I_rxe arrive within a few hundred
     * microseconds of each other at 106kbit/s and both land in the same
     * sticky mask - waiting for them with two sequential reads meant the
     * I_txe wait consumed I_rxe and the I_rxe wait then hung until timeout,
     * reporting "no tag" for every tag. */
    uint32_t irq = 0;
    esp_err_t err = direct_cmd(DCMD_TX_REQA);
    if (err == ESP_OK) {
        err = wait_response(&irq, deadline);
    }
    if (err != ESP_OK) {
        /* No tag in range - not a driver error, just nothing to report. */
        return (err == ESP_ERR_TIMEOUT) ? ESP_ERR_NOT_FOUND : err;
    }
    size_t n = 0;
    ESP_RETURN_ON_ERROR(fifo_byte_count(&n), TAG, "fifo count (atqa)");
    if (n < 2) {
        /* Something answered but it isn't a 2-byte ATQA - don't walk into the
         * anticollision cascade on a frame we didn't understand. */
        ESP_LOGW(TAG, "short ATQA: %u bytes in FIFO", (unsigned)n);
        return ESP_ERR_INVALID_RESPONSE;
    }
    uint8_t atqa[2] = { 0 };
    ESP_RETURN_ON_ERROR(fifo_read(atqa, 2), TAG, "fifo read atqa");

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

/* ---- Type 2 Tag memory read (NTAG21x / MIFARE Ultralight family) ----
 *
 * Only valid while a tag is selected - i.e. right after st25r3916_try()
 * returns success, before st25r3916_close(). cascade_level()'s SELECT step
 * already left antcl off and the tag in the ISO14443-3 ACTIVE state, so a
 * plain CRC-framed command here (same shape as SELECT's own TX) is exactly
 * what a selected Type 2 Tag expects. */

/* READ (0x30 + 1-byte page address) -> 16 bytes (4 pages), CRC-checked and
 * stripped by the chip the same way the SAK response already is. Mirrors
 * cascade_level()'s structure: Clear FIFO (also resets IRQ status, SS4.3.1)
 * -> load the command -> Transmit with CRC -> wait on the sticky IRQ mask
 * -> read back whatever the FIFO actually holds. */
static esp_err_t type2_read_pages(uint8_t page, uint8_t out16[16], TickType_t deadline)
{
    ESP_RETURN_ON_ERROR(direct_cmd(DCMD_CLEAR_FIFO), TAG, "clear fifo (type2 read)");
    uint8_t cmd[2] = { 0x30, page };
    ESP_RETURN_ON_ERROR(fifo_load(cmd, sizeof(cmd)), TAG, "fifo load (type2 read)");
    ESP_RETURN_ON_ERROR(reg_write1(REG_NUM_TX_BYTES1, 0x00), TAG, "ntx1 (type2 read)");
    ESP_RETURN_ON_ERROR(reg_write1(REG_NUM_TX_BYTES2, (uint8_t)(2u << 3)), TAG, "ntx2 (type2 read)");

    uint32_t irq = 0;
    ESP_RETURN_ON_ERROR(direct_cmd(DCMD_TX_WITH_CRC), TAG, "tx type2 read");
    esp_err_t err = wait_response(&irq, deadline);
    if (err != ESP_OK) {
        /* Expected at the tag's memory boundary (some tags NAK a
         * out-of-range page instead of wrapping) - not logged as a warning
         * here, the caller decides whether a short read is normal. */
        return err;
    }
    size_t n = 0;
    ESP_RETURN_ON_ERROR(fifo_byte_count(&n), TAG, "fifo count (type2 read)");
    if (n < 16) {
        return ESP_ERR_INVALID_RESPONSE;   /* NAK (short response) or truncated read */
    }
    return fifo_read(out16, 16);
}

esp_err_t st25r3916_read_type2(uint8_t *out, size_t out_cap, size_t *out_len, int timeout_ms)
{
    if (!out || !out_len || out_cap == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    *out_len = 0;
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);

    /* Page 0-3 in one READ: pages 0-1 are the UID, page 2 lock/internal
     * bytes, page 3 the Capability Container. Only the CC's magic number
     * (byte 0 of page 3 = out16[12]) is checked here - if it isn't 0xE1 the
     * tag was never NDEF-formatted, and walking its TLV area would just be
     * parsing whatever happens to be there. Not an error: a tag with no
     * NDEF content at all is a normal, expected outcome. */
    uint8_t hdr16[16];
    if (type2_read_pages(0, hdr16, deadline) != ESP_OK) {
        return ESP_ERR_NOT_FOUND;
    }
    if (hdr16[12] != 0xE1) {
        return ESP_ERR_NOT_FOUND;
    }

    /* User memory / TLV area starts at page 4. Read forward in 4-page (one
     * READ command = 16 bytes) chunks until out_cap is filled or a read
     * fails - a short/failed read this far in is the tag's memory boundary,
     * not a driver error, so it ends the read with whatever was gathered
     * rather than propagating an error the caller would have to unwrap. */
    size_t got = 0;
    uint8_t page = 4;
    while (got < out_cap) {
        uint8_t chunk[16];
        if (type2_read_pages(page, chunk, deadline) != ESP_OK) {
            break;
        }
        size_t take = out_cap - got;
        if (take > 16) {
            take = 16;
        }
        memcpy(out + got, chunk, take);
        got += take;
        page = (uint8_t)(page + 4);
    }

    *out_len = got;
    return (got > 0) ? ESP_OK : ESP_ERR_NOT_FOUND;
}

/* Diagnostic primitive: see the doc comment in st25r3916.h for why this
 * uses the permanent SPI device (s_spi, bound once in st25r3916_init())
 * rather than creating a temporary one - a temporary device sharing GPIO4
 * with the permanent one was the leading suspect for the permanent device
 * going permanently unresponsive after a probe run.
 *
 * Deliberately does NOT touch any power rail - the caller owns that, since
 * "which rails are up" is board-level policy and the interesting axes (the
 * NFC rail, the SD card's rail, LoRa's rail) live outside this driver. Only
 * meant to be called with no st25r3916_open() session active - Set default
 * resets whatever RF/mode state a session had configured. Returns true when
 * both reads agree and carry the ST25R3916 type nibble. */
bool st25r3916_probe_identity(uint8_t out_id[2])
{
    out_id[0] = 0;
    out_id[1] = 0;

    direct_cmd(DCMD_SET_DEFAULT);
    vTaskDelay(pdMS_TO_TICKS(2));
    reg_read1(REG_IC_IDENTITY, &out_id[0]);
    reg_read1(REG_IC_IDENTITY, &out_id[1]);

    return ((out_id[0] & IC_IDENTITY_TYPE_MASK) == IC_IDENTITY_TYPE_ST25R3916) &&
           out_id[0] == out_id[1];
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
