# Display power: a proposed owner for panel visibility

Status: **proposal, not implemented.** Written 2026-09-02 while the reasoning
was fresh, after five consecutive broken attempts at the sleep/wake display
path in one session. Implementation deliberately left for a separate, calm
change - see "When to do this".

## The problem

Nothing owns the question "what state is the panel in right now".

`power_mgmt.c` alone has 13 direct panel-command sites, each open-coding its own
sequence:

- `co5300_display_off(); co5300_blank(); co5300_sleep();` - three separate places
- `co5300_reinit(); co5300_set_brightness(); lvgl_force_redraw(); co5300_display_on();`
- `co5300_set_brightness()` on its own, for night mode and the brightness setting

`lvgl_app.c` and `debug_cmds.c` add more, and `lvgl_force_redraw()` is called
from six places across three task contexts.

So every new requirement adds another hand-rolled sequence, executed from
whichever task happened to notice the need. There is no single place where the
ordering rules live, and no way to be sure a given sequence is safe from the
context it runs in.

## What that cost, concretely

Five bugs shipped in one session, all from this one area:

1. **Wake classified from `esp_sleep_get_wakeup_causes()`** - the bitmap reports
   `BIT(ESP_SLEEP_WAKEUP_UNDEFINED)` for timer wakes, never `BIT(TIMER)`. 623
   heartbeats a day each lit the screen.
2. **"Defer unless positively identified" as the default** - touch is handled by
   the adapter's own ISR and mesh arrives via sx1262's DIO1 handler, so neither
   sets a bit this code can see. Both went dark; a LoRa message vibrated with
   the screen off.
3. **Recognising the heartbeat by sleep duration** - sleeps routinely run longer
   than the 60 s period (116 s, 658 s, 3602 s observed), so the test matched
   every touch after a long sleep.
4. **`panel_deferred = imu_only`** - a touch wake usually carries the IMU bit
   too (the BHI chatters constantly), so real touches were detected and then
   discarded. The log showed `src=0x100 touch=1 panel=deferred`.
5. **`co5300_blank()` called from inside the LVGL flush callback** - re-entered
   the panel IO while a flush transaction was in flight, stalled the pipeline so
   LVGL never got its flush-ready, and killed the UI entirely.

Note the shape: (1)-(4) are all attempts to *predict* whether the screen should
be on from the wake source. (5) is panel IO from the wrong context. Both are
symptoms of the same missing structure.

## The proposal

A `display_power` module that is the sole owner of panel visibility.

### States

```
SLEEPING  - panel in SLPIN, display off, GRAM undefined
READY     - panel reset + initialised, display still off, GRAM known
VISIBLE   - display on, brightness applied
```

### API

All non-blocking, callable from any task including the flush callback:

```c
void display_power_show(void);       /* request VISIBLE */
void display_power_hide(void);       /* request SLEEPING */
void display_power_note_draw(void);  /* the UI is drawing - guarantee VISIBLE */
bool display_power_is_visible(void);
```

### Internals

One task, one queue. **That task is the only code in the system that calls
`co5300_*`.** Callers post intent and return; they never touch the panel.

Transitions are written once, with the ordering rules inside them:

- `-> VISIBLE` from `SLEEPING`: reset, init command list, set gap, **blank**,
  brightness, full repaint, then DISPON. Undefined GRAM is never shown - this is
  what the "light green background" bug was.
- `-> SLEEPING`: DISPOFF, blank, SLPIN.
- The white-screen recovery (hardware RST + full init, see
  `docs/` / commit 57fa546) is the same `-> VISIBLE` path, so recovery and wake
  cannot drift apart.

## Why this fixes the class, not the instances

| Bug | Why it becomes impossible |
| --- | --- |
| Panel IO inside the flush callback | The flush path can only post to a queue |
| GRAM garbage on show | Blank-before-show lives inside the one transition |
| Screen stuck dark after a deferral | Any draw notification forces VISIBLE |
| Wake misclassification killing touch | Classification only *delays* showing; it never decides visibility |
| Recovery drifting from wake | Both are the same transition |

## The key design point

Four of the five bugs came from trying to derive "should the screen be on?" from
the wake source. That is a prediction about the future, made with incomplete
information - the code cannot see touch or LoRa wakes at all, because those
paths belong to other drivers' ISRs.

**The UI drawing is ground truth.** It is an observation, not a prediction: if
LVGL is flushing pixels, the screen must be visible, whatever woke it and
whether or not this code understood the wake.

So the abstraction should make drawing authoritative and demote wake
classification to what it actually is - a power optimisation that may delay
turning the display on, and must never be able to prevent it.

## When to do this

**Not while the wake path is unstable.** This refactor touches exactly the code
that has been breaking. Sequence:

1. Confirm the current build is healthy: screen, touch, sleep/wake, a full day
   of normal use.
2. Implement `display_power` with the invariants above, moving all 13+ call
   sites behind it. No behaviour change intended.
3. Only then re-introduce wake-based deferral, as an optimisation on top of a
   base that is correct without it.

### Smaller alternative

If the refactor is not wanted, delete the deferral entirely and always show on
wake. Simpler and known-good, at the cost of IMU chatter lighting the screen for
an idle timeout. That is a better trade than the current machinery, which has
cost far more than it saved.

## Related

- `[[white-screen-root-cause]]` - the RST+init recovery that must share the
  `-> VISIBLE` transition
- `docs/ideas.md` - other pending work
