/*
 * debug_cmds.h - miscellaneous debug console commands (SD, power, alarm,
 * BLE, motor, RTC, crash dump).
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void debug_cmd_shot(const char *args);
void debug_cmd_sdin(const char *args);
void debug_cmd_sdls(const char *args);
void debug_cmd_sdclear(const char *args);
void debug_cmd_heap(const char *args);
void debug_cmd_crashinfo(const char *args);
void debug_cmd_crashsave(const char *args);
void debug_cmd_crashls(const char *args);
void debug_cmd_crashread(const char *args);
void debug_cmd_panictest(const char *args);
void debug_cmd_dailylog(const char *args);
void debug_cmd_motor(const char *args);
void debug_cmd_pm(const char *args);
void debug_cmd_bat(const char *args);
void debug_cmd_dispchk(const char *args);
void debug_cmd_disppwr(const char *args);
void debug_cmd_alarm(const char *args);
void debug_cmd_alarmring(const char *args);
void debug_cmd_alarmdismiss(const char *args);
void debug_cmd_alarmsnooze(const char *args);
void debug_cmd_rtcdump(const char *args);
void debug_cmd_rtctimer(const char *args);
void debug_cmd_settime(const char *args);
void debug_cmd_ble(const char *args);
void debug_cmd_bleadv(const char *args);
void debug_cmd_bleadvoff(const char *args);

#ifdef __cplusplus
}
#endif
