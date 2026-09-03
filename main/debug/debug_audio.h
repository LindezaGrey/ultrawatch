/*
 * debug/debug_audio.h - audio debug console commands (tone/rec/playrec/tonerec/sweep).
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void debug_audio_tone(const char *args);
void debug_audio_rec(const char *args);
void debug_audio_playrec(const char *args);
void debug_audio_tonerec(const char *args);
void debug_audio_sweep(const char *args);

#ifdef __cplusplus
}
#endif
