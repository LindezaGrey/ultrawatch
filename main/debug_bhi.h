/*
 * debug_bhi.h - BHI260AP sensor-hub debug console commands.
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void debug_bhi_bhi(const char *args);
void debug_bhi_suspend(const char *args);
void debug_bhi_resume(const char *args);
void debug_bhi_imon(const char *args);
void debug_bhi_wudump(const char *args);
void debug_bhi_wusus(const char *args);
void debug_bhi_wudis(const char *args);
void debug_bhi_metahist(const char *args);
void debug_bhi_sensorlist(const char *args);
void debug_bhi_bhishow(const char *args);
void debug_bhi_grate(const char *args);

#ifdef __cplusplus
}
#endif
