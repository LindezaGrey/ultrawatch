#!/usr/bin/env python3
"""Compile the actual main-loop telemetry block with failing board readers."""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
source = (root / 'main/main.c').read_text()
start = source.index('        twatch_power_snapshot_t power;', source.index('next_telemetry_ms = now_ms'))
end = source.index('        // USB may enumerate', start)
block = source[start:end]
harness = r'''
#include <assert.h>
#include <inttypes.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "twatch_board.h"
#include "battery_observation.h"
#define TAG "test"
#define BROOKESIA_SYSTEM_RUNTIME 0
#define ESP_ERROR_CHECK(x) assert((x) == ESP_OK)
static int power_error, rails_error, power_calls, rail_calls;
static int power_logs, rail_logs, warning_logs, observations;
static uint32_t power_read_failures, rail_read_failures;
static battery_observation_t battery_run;
static void log_line(const char *tag, const char *fmt, ...) {
    (void)tag;
    if (!strncmp(fmt, "PMU status=", 11)) ++power_logs;
    if (!strncmp(fmt, "RAILS dcdc=", 11)) ++rail_logs;
    if (!strncmp(fmt, "TELEMETRY_READ", 14)) ++warning_logs;
}
#define ESP_LOGI(...) log_line(__VA_ARGS__)
#define ESP_LOGW(...) log_line(__VA_ARGS__)
static const char *esp_err_to_name(int err) { return err ? "TIMEOUT" : "OK"; }
static int64_t esp_timer_get_time(void) { return 1000000; }
esp_err_t twatch_board_read_power(twatch_power_snapshot_t *out) {
    ++power_calls;
    memset(out, power_error ? 0xa5 : 0, sizeof(*out));
    if (!power_error) { out->battery_voltage_available = true; out->battery_mv = 4100; }
    return power_error;
}
esp_err_t twatch_board_read_rails(twatch_rail_snapshot_t *out) {
    ++rail_calls; memset(out, rails_error ? 0xa5 : 0, sizeof(*out)); return rails_error;
}
void battery_observe(battery_observation_t *record, uint64_t now, bool usb, bool present,
                     bool charging, bool valid, uint16_t mv) {
    (void)record; (void)now; (void)usb; (void)present; (void)charging;
    assert(valid && mv == 4100); ++observations;
}
static void sample(void) {
''' + block + r'''
}
int main(void) {
    for (int mask = 0; mask < 4; ++mask) {
        power_error = (mask & 1) ? 0x107 : 0;
        rails_error = (mask & 2) ? 0x107 : 0;
        power_calls = rail_calls = power_logs = rail_logs = warning_logs = observations = 0;
        uint32_t p = power_read_failures, r = rail_read_failures;
        sample();
        assert(power_calls == 1 && rail_calls == 1);
        assert(power_logs == !power_error && rail_logs == !rails_error);
        assert(observations == !power_error && warning_logs == !!mask);
        assert(power_read_failures == p + !!power_error);
        assert(rail_read_failures == r + !!rails_error);
    }
    power_error = rails_error = 0;
    observations = 0;
    sample();
    assert(observations == 1); // successful sampling resumes after both fail
    puts("PASS: all four read outcomes, incomplete snapshots skipped, recovery, bounded calls and counters");
}
'''
with tempfile.TemporaryDirectory(prefix='ultra-telemetry-test-') as tmp:
    work = Path(tmp)
    (work / 'esp_err.h').write_text('#pragma once\ntypedef int esp_err_t;\n#define ESP_OK 0\n')
    (work / 'test.c').write_text(harness)
    subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                    '-I' + str(work), '-I' + str(root / 'components/twatch_board/include'),
                    '-I' + str(root / 'main'), str(work / 'test.c'), '-o', str(work / 'test')], check=True)
    subprocess.run([str(work / 'test')], check=True)
