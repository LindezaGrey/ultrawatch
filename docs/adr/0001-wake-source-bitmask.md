# Track wake sources as a bitmask, not a last-writer-wins scalar

`power_mgmt.c`'s `button_isr` attributes each light-sleep wake to a single GPIO in `s_wake_gpio`, so when two wake sources fire close together (e.g. an alarm and a wrist-gesture), one silently overwrites the other and its source-specific handling (e.g. `alarm_handle_wake()`) is skipped until the periodic safety net catches it, up to ~5 s later. We're replacing `s_wake_gpio` with a bitmask (`s_wake_sources`), OR'd in from the ISR under a critical section, so every source that fired gets its own handling in the same wake-processing pass. Touch is excluded — it has no source-specific post-wake behavior, so there's nothing for attribution to buy it.

## Considered Options

- **Keep the single scalar, rely on the safety net**: rejected — the ~5 s alarm latency is the exact bug being fixed, not an acceptable fallback.
- **A queue/list of wake events**: rejected as unnecessary — the source set is small (4) and fixed, and within one wake cycle a source can only usefully fire once, so a bitmask carries the same information as a queue without the extra structure.
