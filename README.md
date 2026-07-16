# RPLidar for Grasshopper

Slamtec RPLIDAR integration for Grasshopper. Connect a 2D LiDAR over serial, stream 360-degree scans into Rhino as points, and extract room wall outlines in real time.

## Components

| Component | Nickname | Description |
|-----------|----------|-------------|
| **RPLIDAR Device** | RPLIDAR | Connects to an RPLIDAR and streams 360-degree scans. |
| **RPLIDAR Scan** | Scan | Reads the latest 360-degree scan from the device as points. |
| **Outliner** | Outliner | Extracts a room wall outline from a single sweep using an order-aware, deterministic split-and-merge (IEPF) algorithm - no RANSAC. |

## How it works

Place the **RPLIDAR Device** component and give it the serial port your LiDAR is on. **RPLIDAR Scan** reads the current sweep as a set of points, with the sensor at the world origin. **Outliner** turns a single ordered sweep into clean wall segments: it splits the scan into runs at range/angle gaps, fits straight segments with Iterative End-Point Fit (IEPF), and merges near-collinear pieces.

## Finding the serial port

Use the separate **SerialScanner** plugin to list connected serial devices and find your RPLIDAR's COM port.

## Installation

**Rhino Package Manager (recommended)**

1. In Rhino 8, run `_PackageManager`.
2. Search for **rplidar** and click Install.

**Manual**

Download the `.yak` from the [latest release](https://github.com/AndrewJinhoAhn/RPLidarGH/releases) and install it through the Package Manager.

## Requirements

- Rhino 8 (Windows)
- A Slamtec RPLIDAR connected over USB/serial