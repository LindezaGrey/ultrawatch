/*
 * debug_gnss.h - GNSS/tracking debug console commands.
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void debug_gnss_on(const char *args);
void debug_gnss_off(const char *args);
void debug_gnss_ver(const char *args);
void debug_gnss_rtccal(const char *args);
void debug_gnss_status(const char *args);
void debug_gnss_check(const char *args);
void debug_gnss_seed(const char *args);
void debug_gnss_lpk(const char *args);
void debug_gnss_raw(const char *args);
void debug_gnss_probe(const char *args);
void debug_gnss_track(const char *args);
void debug_gnss_cachedump(const char *args);
void debug_gnss_trackstat(const char *args);

#ifdef __cplusplus
}
#endif
