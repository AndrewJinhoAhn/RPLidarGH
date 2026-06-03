# RPLidarGH

RPLidar integration for Rhino Grasshopper — a native C# plugin that connects to an **RPLidar C1** 2D 360° LiDAR over USB serial, streams live scans into Grasshopper, and reconstructs indoor wall outlines.

No Slamtec SDK required: the serial protocol (Slamtec *S & C Series Interface Protocol*) is implemented directly in C#.

## Components

All under the **Appendage › RPLiDAR** tab.

### RPLIDAR Device
Connects to the sensor and runs a background thread that continuously assembles 360° rotations.
- **Inputs:** `Active` (on/off), `Port` (e.g. `COM7`), `RPM` (motor speed; 600 ≈ 10 Hz)
- **Outputs:** `Lidar` (device handle → feed to Scan), `Status`

### RPLIDAR Scan
Pulls the latest completed rotation from the device handle.
- **Input:** `Lidar` (from Device)
- **Outputs:** `Points`, `Distances`, `Angles`, `Count`, `ScanHz`
- Drive it with a Grasshopper **Timer** for live updates.

### Outliner
Reconstructs a wall outline polyline from a 2D point cloud: sequential RANSAC wall detection (distance/incidence-adaptive) → duplicate-wall merge → greedy endpoint assembly → angle-branched corners.
- **Inputs:** `Points`, `Threshold`, `MinInliers`, `MinLength`, `AngleDeg`
- **Outputs:** `Outline`, `Walls`, `Corners`, `WallPoints`, `Info`

## Requirements
- Rhino 8 (Windows) — also builds for Rhino 7 (`net48`)
- RPLidar C1 + USB adapter
- **Silicon Labs CP210x VCP driver** (the C1 adapter uses a CP2102N) — https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers

## Hardware setup
1. Install the CP210x VCP driver and plug in the C1.
2. Find the COM port in Device Manager (Silicon Labs CP210x → e.g. `COM7`).
3. In Grasshopper, set the **Device** component's `Port` to that COM port and toggle `Active` on.

The C1 runs at 460800 baud, 10 Hz / 5000 samples-per-second, 0.72° resolution, 12 m range.

## Usage

```
Device (Active, Port, RPM)
   └─► Scan (Points, ScanHz, …)
          └─► Outliner (Outline, Walls, …)
Timer ──► Scan        (drives live refresh)
```

## Build
```
dotnet build
```
Targets `net8.0-windows;net48` and produces a `.gha`. Dependencies: `Grasshopper`, `System.IO.Ports`.

## Install
Copy the built `.gha` into your Grasshopper Libraries folder:
```
%AppData%\Grasshopper\Libraries\
```
If Windows flagged the file, right-click the `.gha` → Properties → **Unblock**, then restart Rhino.

## Other RPLidar models
The S & C series share this serial protocol, so the plugin works on S1/S2/S3 after changing the baud rate (S1 = 256000, S2/S3 = 1000000). Note that standard SCAN runs the S-series at a reduced sample rate; their full density requires EXPRESS_SCAN (dense/capsuled) decoding, which is not yet implemented.

## License
TODO — add a license (e.g. MIT).
