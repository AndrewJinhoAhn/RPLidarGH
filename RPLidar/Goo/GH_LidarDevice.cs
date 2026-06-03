using System;
using Grasshopper.Kernel.Types;

namespace RPLidar
{
    /// <summary>
    /// Grasshopper goo wrapping a live RPLIDAR connection.
    ///
    /// Mirrors GH_KinectDevice from the Azure Kinect plugin: the Value is just
    /// an identity string (the serial port name); the useful payload is the
    /// <see cref="AcquireLatestScan"/> delegate, which downstream components
    /// call on their own solve to pull the most recent completed rotation.
    ///
    /// Reference semantics: duplicating this goo does NOT open a new connection.
    /// All wrappers point at the same underlying serial reader thread.
    /// </summary>
    public class GH_LidarDevice : GH_Goo<string>
    {
        /// <summary>
        /// Returns the latest completed 360-degree rotation, or null if none
        /// has been assembled yet. The returned frame is immutable, so no
        /// disposal or copying is required (cf. Kinect's DuplicateReference()).
        /// Set by LidarDeviceComponent when it creates this handle.
        /// </summary>
        public Func<LidarFrame> AcquireLatestScan { get; set; }

        public GH_LidarDevice() : base() { }
        public GH_LidarDevice(string portName) : base(portName) { }

        public override bool IsValid => !string.IsNullOrEmpty(Value) && AcquireLatestScan != null;

        public override string TypeName => "RPLIDAR Device";
        public override string TypeDescription =>
            "Live RPLIDAR connection handle (serial port + latest-scan accessor).";

        public override IGH_Goo Duplicate()
        {
            // Share the same reader thread — reference semantics.
            return new GH_LidarDevice
            {
                Value = this.Value,
                AcquireLatestScan = this.AcquireLatestScan,
            };
        }

        public override string ToString()
        {
            if (string.IsNullOrEmpty(Value)) return "RPLIDAR Device (null)";
            return IsValid ? $"RPLIDAR Device ({Value})" : "RPLIDAR Device (inactive)";
        }
    }
}