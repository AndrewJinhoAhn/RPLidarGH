using System;

namespace RPLidar
{
    /// <summary>
    /// A single measurement node from the RPLIDAR scan stream.
    /// Angle is in degrees [0, 360), distance in millimeters
    /// (0 means an invalid / no-return measurement).
    /// </summary>
    public readonly struct LidarPoint
    {
        public readonly double AngleDeg;
        public readonly double DistanceMm;
        public readonly int Quality;

        public LidarPoint(double angleDeg, double distanceMm, int quality)
        {
            AngleDeg = angleDeg;
            DistanceMm = distanceMm;
            Quality = quality;
        }
    }

    /// <summary>
    /// One complete 360-degree rotation's worth of measurement nodes.
    /// Immutable snapshot: once handed out it is never mutated, so it can be
    /// shared across threads without copying.
    /// </summary>
    public sealed class LidarFrame
    {
        public readonly LidarPoint[] Points;
        public readonly DateTime TimestampUtc;
        public readonly double ScanHz;   // measured actual rotation frequency (0 = not known yet)

        public LidarFrame(LidarPoint[] points, double scanHz = 0)
        {
            Points = points ?? Array.Empty<LidarPoint>();
            TimestampUtc = DateTime.UtcNow;
            ScanHz = scanHz;
        }

        public int Count => Points.Length;
    }
}