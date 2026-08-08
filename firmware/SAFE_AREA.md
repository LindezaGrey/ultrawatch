# Display safe drawing area

The display coordinate system is `x = 0..409`, `y = 0..501`.

The blue line in the validation firmware is a calibration contour. **No blue
pixel is part of the safe drawing area.** The safe area is only the black
interior enclosed by that line. Black pixels outside the contour are also not
safe.

Use `pixel_is_safe_content(x, y)` in `main/app_main.c` as the authoritative
test. A rectangular bounds check is insufficient because the four corners are
curved.

## Calibrated values

| Value | Pixels | Source |
|---|---:|---|
| Outer edge of blue contour | 13 px inward | Visual calibration |
| Blue contour width | 3 px | Validation pattern |
| Safe-content inset | 16 px | `13 + 3`; the blue line is excluded |
| Vertical contour offset | 1 px down | Visual calibration |
| Additional top-corner curvature | 3 px | Visual calibration |

The resulting safe mask has these measured properties under the firmware's
integer calculations:

- bounding box: `x = 16..393`, `y = 17..486` (378 x 470 pixels);
- safe pixels: 170,167;
- first safe row (`y = 17`): `x = 98..311`;
- middle row (`y = 251`): `x = 16..393`;
- last safe row (`y = 486`): `x = 102..307`.

The bounding box describes only the mask's extent. It does not make every pixel
inside that rectangle safe; the corner equations below remain authoritative.

## Calculation

The physical active area and nominal corner radii are defined in `main/board.h`:

```text
W = 33090 um, H = 40510 um
display = 410 x 502 pixels
top radius = 8420 um
bottom radius = 9000 um
```

For display pixel `(x, y)`, first apply the one-pixel downward placement of the
shape:

```text
shape_y = y - 1
```

`shape_y < 0` is unsafe. Otherwise, map the pixel center to physical units using
integer division:

```text
X = ((2*x + 1) * 33090) / (2*410)
Y = ((2*shape_y + 1) * 40510) / (2*502)
```

The safe-content inset and top-radius correction are:

```text
I = (16 * 33090) / 410 = 1291 um
E = (3 * 33090) / 410 = 242 um
```

A pixel must first satisfy:

```text
I <= X <= W-I
I <= Y <= H-I
```

For the top corners:

```text
top center offset Ctop = 8420 + E = 8662 um
safe top radius Rtop   = 8420 - I + E = 7371 um
```

Pixels in a top-corner quadrant are safe only when their squared distance from
the applicable center `(Ctop, Ctop)` or `(W-Ctop, Ctop)` is no greater than
`Rtop^2`.

For the bottom corners:

```text
bottom centers = (9000, H-9000) and (W-9000, H-9000)
safe bottom radius Rbottom = 9000 - I = 7709 um
```

Pixels in a bottom-corner quadrant are safe only when their squared distance
from the applicable center is no greater than `Rbottom^2`. Pixels that pass the
inset bounds and are not in a corner quadrant are safe.

The displayed blue contour is the set difference between the same geometry at
13 px and at 16 px. Therefore the 3-pixel blue line is excluded by construction:

```text
safe(x, y) = geometry(x, y-1, inset=16)
blue(x, y) = geometry(x, y-1, inset=13) AND NOT safe(x, y)
```
