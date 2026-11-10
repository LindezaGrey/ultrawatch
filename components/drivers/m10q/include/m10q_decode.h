#pragma once
#include <stddef.h>
#include "m10q_types.h"

/* Pure payload decoders. Truncated/unsupported messages leave the cache intact. */
bool m10q_decode_sat(m10q_fix_t *fix, const uint8_t *p, size_t length, uint32_t now);
bool m10q_decode_pvt(m10q_fix_t *fix, const uint8_t *p, size_t length, uint32_t now);
bool m10q_decode_dop(m10q_fix_t *fix, const uint8_t *p, size_t length, uint32_t now);
void m10q_expire_snapshot(m10q_fix_t *fix, uint32_t now);
