/*
 * crash_dump.c - decode a crash core dump from flash and save it to SD.
 *
 * On panic, ESP-IDF's espcoredump stores an ELF core dump in the "coredump"
 * flash partition. On the next boot we decode it into a readable text report
 * (panic reason, crashed task, registers, backtrace) and also copy the raw ELF
 * to /sdcard/log/crash/ so it can be analyzed offline with espcoredump.py.
 *
 * The flash copy is erased only after a successful save, so a dump survives if
 * the SD card is missing at the next boot.
 */
#include "crash_dump.h"

#include <stdio.h>
#include <string.h>
#include <time.h>
#include <sys/stat.h>
#include <sys/unistd.h>

#include "esp_log.h"
#include "esp_core_dump.h"
#include "esp_partition.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "sd_log.h"

static const char *TAG = "crash_dump";

#define CRASH_DIR "/sdcard/log/crash"

/* Xtensa exception cause names (EXCCAUSE register), index = cause number.
 * Matches the table in esp_system panic_arch.c. */
static const char *const k_exc_cause_names[64] = {
    [0] = "IllegalInstruction", [1] = "Syscall", [2] = "InstructionFetchError",
    [3] = "LoadStoreError",     [4] = "Level1Interrupt", [5] = "Alloca",
    [6] = "IntegerDivideByZero",[7] = "PCValue", [8] = "Privileged",
    [9] = "LoadStoreAlignment",
    [12] = "InstrPDAddrError",  [13] = "LoadStorePIFDataError",
    [14] = "InstrPIFAddrError", [15] = "LoadStorePIFAddrError",
    [16] = "InstTLBMiss",       [17] = "InstTLBMultiHit",
    [18] = "InstFetchPrivilege",
    [20] = "InstrFetchProhibited",
    [24] = "LoadStoreTLBMiss",  [25] = "LoadStoreTLBMultihit",
    [26] = "LoadStorePrivilege",
    [28] = "LoadProhibited",    [29] = "StoreProhibited",
    [32] = "Cp0Dis",            [33] = "Cp1Dis", [34] = "Cp2Dis",
    [35] = "Cp3Dis",            [36] = "Cp4Dis", [37] = "Cp5Dis",
    [38] = "Cp6Dis",            [39] = "Cp7Dis",
};

static void make_timestamp(char *buf, size_t len)
{
    /* System clock is synced from the RTC at boot; TZ is set in app_main. */
    time_t now = time(NULL);
    struct tm tmv;
    localtime_r(&now, &tmv);
    strftime(buf, len, "%Y%m%d_%H%M%S", &tmv);
}

/* Copy the raw ELF core dump from the coredump flash partition to a file.
 * Returns the number of bytes copied, or 0. */
static size_t save_raw_elf(const char *path)
{
    const esp_partition_t *part =
        esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_COREDUMP, NULL);
    if (!part) {
        ESP_LOGW(TAG, "no coredump partition");
        return 0;
    }

    size_t addr = 0, size = 0;
    if (esp_core_dump_image_get(&addr, &size) != ESP_OK || size == 0) {
        ESP_LOGW(TAG, "no core dump image in flash");
        return 0;
    }

    FILE *f = fopen(path, "wb");
    if (!f) {
        ESP_LOGW(TAG, "cannot open %s", path);
        return 0;
    }

    /* addr is an absolute flash offset; esp_partition_read needs the offset
     * relative to the partition start. */
    size_t rel = addr - part->address;
    size_t left = size;
    uint8_t *chunk = heap_caps_malloc(4096, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!chunk) {
        chunk = malloc(4096);
    }
    size_t written = 0;
    while (left > 0) {
        size_t n = (left > 4096) ? 4096 : left;
        if (esp_partition_read(part, rel, chunk, n) != ESP_OK) {
            ESP_LOGE(TAG, "partition read failed at %u", (unsigned)rel);
            break;
        }
        fwrite(chunk, 1, n, f);
        rel += n;
        left -= n;
        written += n;
    }
    free(chunk);
    fclose(f);
    return written;
}

/* Write a human-readable crash report. Returns true on success. */
static bool write_report(const char *path)
{
    esp_core_dump_summary_t *sum = malloc(sizeof(esp_core_dump_summary_t));
    if (!sum) {
        ESP_LOGE(TAG, "no mem for summary");
        return false;
    }
    if (esp_core_dump_get_summary(sum) != ESP_OK) {
        ESP_LOGE(TAG, "failed to read core dump summary");
        free(sum);
        return false;
    }

    char reason[256] = { 0 };
    esp_core_dump_get_panic_reason(reason, sizeof(reason));

    uint32_t cause = sum->ex_info.exc_cause;
    const char *cause_name = (cause < 64 && k_exc_cause_names[cause]) ? k_exc_cause_names[cause] : "unknown";

    FILE *f = fopen(path, "w");
    if (!f) {
        free(sum);
        ESP_LOGW(TAG, "cannot open %s", path);
        return false;
    }

    /* For plain exceptions (deref faults, etc.) the coredump has no panic
     * details note; derive a readable reason from the exception cause. */
    char derived[96];
    snprintf(derived, sizeof(derived), "Guru Meditation Error: %s", cause_name);

    fprintf(f, "UWatch crash report\n");
    fprintf(f, "===================\n\n");
    fprintf(f, "App build:       %s\n", CONFIG_APP_PROJECT_VER);
    fprintf(f, "App ELF SHA256:  %s\n", (char *)sum->app_elf_sha256);
    fprintf(f, "Panic reason:    %s\n", reason[0] ? reason : derived);
    fprintf(f, "Crashed task:    %s (tcb=0x%x)\n", sum->exc_task, (unsigned)sum->exc_tcb);
    fprintf(f, "Exception PC:    0x%08x\n", (unsigned)sum->exc_pc);

    fprintf(f, "Exception cause: %lu (%s)\n", (unsigned long)cause, cause_name);
    fprintf(f, "Exception vaddr: 0x%08x\n", (unsigned)sum->ex_info.exc_vaddr);

    fprintf(f, "\nRegister dump:\n");
    for (int i = 0; i < 16; i += 4) {
        fprintf(f, "  a%-2d 0x%08x  a%-2d 0x%08x  a%-2d 0x%08x  a%-2d 0x%08x\n",
                i,     (unsigned)sum->ex_info.exc_a[i],
                i + 1, (unsigned)sum->ex_info.exc_a[i + 1],
                i + 2, (unsigned)sum->ex_info.exc_a[i + 2],
                i + 3, (unsigned)sum->ex_info.exc_a[i + 3]);
    }

    fprintf(f, "\nBacktrace (%s):\n",
            sum->exc_bt_info.corrupted ? "corrupted" : "ok");
    for (uint32_t i = 0; i < sum->exc_bt_info.depth && i < 16; i++) {
        fprintf(f, "  0x%08x\n", (unsigned)sum->exc_bt_info.bt[i]);
    }

    fprintf(f, "\nTo symbolize the backtrace, run from the firmware directory:\n");
    fprintf(f, "  python $IDF_PATH/components/espcoredump/espcoredump.py info_corefile\n"
               "    -m build/UWatch.elf -c crash/core_<timestamp>.elf\n");
    fprintf(f, "\nReport generated by UWatch crash_dump on boot.\n");

    fclose(f);
    free(sum);
    ESP_LOGI(TAG, "crash report written to %s", path);
    return true;
}

esp_err_t crash_dump_save(void)
{
    /* No core dump in flash? Nothing to do. */
    esp_err_t err = esp_core_dump_image_check();
    if (err == ESP_ERR_NOT_FOUND) {
        return ESP_OK;
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "core dump image check failed: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGW(TAG, "core dump found in flash, decoding to SD card");

    if (!sd_log_available()) {
        ESP_LOGW(TAG, "no SD card; keeping core dump in flash for next boot");
        return ESP_ERR_NOT_FOUND;
    }

    mkdir(CRASH_DIR, 0755);

    char ts[32];
    make_timestamp(ts, sizeof(ts));

    char report_path[128];
    snprintf(report_path, sizeof(report_path), "%s/report_%s.txt", CRASH_DIR, ts);
    bool report_ok = write_report(report_path);

    char elf_path[128];
    snprintf(elf_path, sizeof(elf_path), "%s/core_%s.elf", CRASH_DIR, ts);
    size_t n = save_raw_elf(elf_path);
    ESP_LOGI(TAG, "raw core dump: %u bytes -> %s", (unsigned)n,
             n ? elf_path : "(failed)");

    if (report_ok && n > 0) {
        err = esp_core_dump_image_erase();
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "flash core dump erased");
        } else {
            ESP_LOGW(TAG, "flash core dump erase failed: %s", esp_err_to_name(err));
        }
    }
    return ESP_OK;
}

/* Console helper: report whether a core dump is pending and where. */
void crash_dump_print_status(void)
{
    esp_err_t err = esp_core_dump_image_check();
    if (err == ESP_ERR_NOT_FOUND) {
        printf("crash: no core dump pending\n");
        return;
    }
    if (err != ESP_OK) {
        printf("crash: core dump check failed: %s\n", esp_err_to_name(err));
        return;
    }
    size_t addr = 0, size = 0;
    esp_core_dump_image_get(&addr, &size);
    printf("crash: core dump pending at flash 0x%x (%u bytes); will save on next clean boot with SD\n",
           (unsigned)addr, (unsigned)size);
}
