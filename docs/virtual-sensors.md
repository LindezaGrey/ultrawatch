# BHI260AP virtual sensors

The Bosch `BHI260AP.fw` image reports virtual sensor availability through
`BHY2_PARAM_SYS_VIRT_SENSOR_PRESENT` (`0x011F`). After firmware boot, the
T-Watch Ultra reads this 256-bit presence map without enabling any stream.

Device: LilyGO T-Watch Ultra V1.0, BHI260AP at I²C `0x28`  
Firmware: Bosch `BHI260AP.fw`, SHA-256
`318def511fd8eb762bf1e61cf02cfd6ecac70f8627d3bacfa7d24a1641f9e915`  
Observed boot status: `0x38`  
Observed available sensors: **33**

The names and functions below come from the vendored Bosch SensorAPI
definitions in [`bhy2_defs.h`](../../components/bhi260ap_core/include/bhy2_defs.h).
“Wake-up” variants are low-power sensors intended to remain usable as a wake
source. Availability does not mean that a sensor is currently configured,
running, or connected to a physical sensing element on this board.

| ID | Bosch name | Function |
|---:|---|---|
| 1 | Accelerometer passthrough | Raw accelerometer data routed with minimal processing. |
| 3 | Accelerometer uncalibrated | Accelerometer output without offset correction. |
| 4 | Accelerometer corrected | Calibrated accelerometer output. |
| 5 | Accelerometer offset | Estimated accelerometer bias/offset. |
| 6 | Accelerometer corrected wake up | Calibrated accelerometer data for wake-up operation. |
| 7 | Accelerometer uncalibrated wake up | Uncalibrated accelerometer data for wake-up operation. |
| 10 | Gyroscope passthrough | Raw gyroscope data routed with minimal processing. |
| 12 | Gyroscope uncalibrated | Gyroscope output without bias correction. |
| 13 | Gyroscope corrected | Calibrated gyroscope output. |
| 14 | Gyroscope offset | Estimated gyroscope bias/offset. |
| 15 | Gyroscope wake up | Gyroscope data for wake-up operation. |
| 16 | Gyroscope uncalibrated wake up | Uncalibrated gyroscope data for wake-up operation. |
| 28 | Gravity vector | Estimated gravity direction and magnitude. |
| 29 | Gravity vector wake up | Gravity estimate for wake-up operation. |
| 31 | Linear acceleration | Acceleration with gravity removed. |
| 32 | Linear acceleration wake up | Gravity-compensated acceleration for wake-up operation. |
| 37 | Game rotation vector | Orientation quaternion optimized without magnetic north reference. |
| 38 | Game rotation vector wake up | Game rotation vector for wake-up operation. |
| 48 | Tilt detector | Reports tilt state changes. |
| 50 | Step detector | Reports individual detected steps. |
| 52 | Step counter | Accumulated step count. |
| 53 | Step counter wake up | Step count usable during wake-up operation. |
| 55 | Significant motion | Reports a significant motion event. |
| 57 | Wake gesture | Reports a gesture suitable for waking the device. |
| 59 | Glance gesture | Reports a glance-like wrist gesture. |
| 61 | Pickup gesture | Reports a pickup/lift gesture. |
| 63 | Activity recognition | Classifies activity such as still, walking, running, cycling, or vehicle. |
| 67 | Wrist tilt gesture | Reports wrist tilt gestures. |
| 69 | Device orientation | Classifies the device’s physical orientation. |
| 70 | Device orientation wake up | Orientation classification for wake-up operation. |
| 75 | Stationary detect | Reports that the device is stationary. |
| 77 | Motion detect | Reports that motion is present. |
| 94 | Step detector wake up | Individual step events for wake-up operation. |

The inventory was captured from the device after flashing commit `e3c80cf`;
the raw evidence is recorded in [`bhi260ap.md`](bringup/bhi260ap.md).

For accelerometer IDs 1, 3–7, and 91, Bosch's reference scaling is
`1/4096 g` per raw count. The Diagnostics page converts the corrected ID 4
sample to milli-g using this factor.
