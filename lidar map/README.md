# RPLIDAR C1 Live 2D Map

A native macOS viewer for the RPLIDAR C1. The app talks to the SLAMTEC SDK directly through `grabScanDataHq()`; it does not parse `ultra_simple` terminal output.

## Hardware defaults

- Serial device: auto-detected from `/dev/cu.usbserial-110` or `/dev/cu.usbserial-10` (override with `RPLIDAR_PORT`)
- Baud rate: `460800`
- SDK: `~/rplidar_sdk` (SDK 2.1.0 with C1 support)

## Build and launch

```bash
cd ~/Desktop/'lidar map'
make
python3 lidar_map.py
```

Click **START SCAN** after the window opens. Use the mouse wheel to zoom, drag to pan, and double-click to reset the view. **STOP SCAN** stops only this app's process.

Click **CAPTURE MAP** to save the current scan as an SVG map and a matching CSV file. The CSV includes angle, distance, quality, and calculated `x_m` / `y_m` coordinates. The viewer uses a 2D coordinate system centered on the lidar: X is left/right, Y is forward/back, and the origin is `(0, 0)`. The C1 is a 2D planar scanner, so Z/height cannot be measured by this sensor.

## Persistent mapping while moving

Click **START MAPPING** after starting the scan. The app keeps accumulated points in a world frame and uses iterative closest-point (ICP) scan matching to estimate X/Y/rotation as you move the lidar. Move the lidar slowly and avoid moving objects where possible. **CAPTURE MAP** exports the accumulated map when mapping is active. **CLEAR STORED MAP** starts a new map.

This is handheld 2D scan-matching SLAM without wheel odometry or an IMU. It is designed for slowly walking or rotating the lidar through a room, but drift can accumulate over long movements and there is no automatic loop closure yet. For best results, keep the lidar level, move slowly, and make occasional returns to previously seen walls. The mapper uses NumPy and SciPy; install them with `python3 -m pip install numpy scipy` if your Python does not already include them.

## Notes

The bridge reports native HQ scan nodes as JSON internally, with angle in degrees, distance in metres, and quality. It uses the SDK's device-info handshake, motor control, scan start, frame acquisition, and orderly stop sequence. The existing SDK and its binaries are left untouched.

If the sensor is already owned by another process, stop that process yourself first; this app does not terminate unrelated sensor processes.
