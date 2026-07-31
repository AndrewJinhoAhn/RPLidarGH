using System;
using System.Collections.Generic;
using System.IO.Ports;
using System.Threading;
using Grasshopper.Kernel;

namespace RPLidar.Components
{
    /// <summary>
    /// Connects to an RPLIDAR C1 over its USB serial adapter and runs a
    /// background thread that continuously assembles 360-degree rotations.
    /// Downstream components pull the latest completed rotation via the
    /// delegate carried on the emitted GH_LidarDevice goo.
    ///
    /// Architecture mirrors KinectDeviceComponent: device lifetime spans many
    /// solves (member fields, not locals), a background loop owns acquisition,
    /// and a latest-wins buffer under a lock decouples the sensor's fixed 10Hz
    /// from Grasshopper's irregular solve timing.
    /// </summary>
    public class LidarDeviceComponent : GH_Component
    {
        // C1 is fixed at 460800 baud (unlike A1 115200 / A2 256000).
        private const int C1_BAUD = 460800;

        // ─── State that persists across SolveInstance calls ────────────────
        private SerialPort _port;
        private string _portName;
        private int _rpm = 600;
        private int _prevRpm = -1;
        private DateTime _lastRotUtc = DateTime.MinValue;
        private double _hzEma = 0;

        // ─── Background capture thread ─────────────────────────────────────
        private Thread _captureThread;
        private CancellationTokenSource _captureCancellation;
        private LidarFrame _latestScan;
        private readonly object _scanLock = new object();

        public LidarDeviceComponent()
          : base("RPLIDAR Device", "RPLIDAR",
                 "Connects to an RPLIDAR C1 and streams 360-degree scans.",
                 "Appendage", "RPLIDAR")
        {
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddBooleanParameter(
                "Active", "On",
                "Activate / deactivate the device. Toggling off stops the motor and closes the port.",
                GH_ParamAccess.item, false);

            pManager.AddTextParameter(
                "Port", "Port",
                "Serial port name of the RPLIDAR adapter (e.g. COM3). Check Device Manager (CP210x).",
                GH_ParamAccess.item, "COM3");
            pManager.AddIntegerParameter(
            "RPM", "RPM",
            "Motor speed in RPM (600=10Hz default, 0=device default). ~480–1200 typical.",
            GH_ParamAccess.item, 600);
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter(
                "Lidar", "Dev",
                "Live RPLIDAR device handle. Pass to the Scan component.",
                GH_ParamAccess.item);

            pManager.AddTextParameter(
                "Status", "Status",
                "Current device status or error message.",
                GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            bool active = false;
            string portName = "COM3";
            DA.GetData(0, ref active);
            DA.GetData(1, ref portName);
            DA.GetData(2, ref _rpm);

            string status;
            try
            {
                if (active && _port == null)
                {
                    StartDevice(portName);
                    status = $"Scanning on {portName} @ {C1_BAUD}";
                }
                else if (!active && _port != null)
                {
                    StopDevice();
                    status = "Stopped";
                }
                else if (active && _port != null && portName != _portName)
                {
                    // Port changed while active: reconnect.
                    StopDevice();
                    StartDevice(portName);
                    status = $"Reconnected on {portName}";
                }
                else
                {
                    status = _port != null ? "Scanning" : "Inactive";
                }
            }
            catch (Exception ex)
            {
                status = $"Error: {ex.Message}";
                StopDevice();
            }

            if (_port != null)
            {
                DA.SetData(0, new GH_LidarDevice(_portName)
                {
                    AcquireLatestScan = AcquireLatestScan,
                });
            }
            if (_port != null && _rpm > 0 && _rpm != _prevRpm)
            {
                try { SetMotorRpm(_rpm); } catch { }
                _prevRpm = _rpm;
            }
            DA.SetData(1, status);
        }

        // ─── Lifecycle helpers ─────────────────────────────────────────────

        private void StartDevice(string portName)
        {
            _port = new SerialPort(portName, C1_BAUD, Parity.None, 8, StopBits.One)
            {
                ReadTimeout = 2000,
                WriteTimeout = 1000,
            };
            _port.Open();
            _port.DtrEnable = false;          // motor on (may be meaningless on the C1 line but harmless)

            SendCommand(0x25);                // STOP -> clean idle
            Thread.Sleep(20);
            _port.DiscardInBuffer();

            if (_rpm > 0)                     // set speed after STOP and before SCAN (SDK order)
            {
                SetMotorRpm(_rpm);
                _prevRpm = _rpm;
                Thread.Sleep(50);
            }

            SendCommand(0x20);                // SCAN
            ReadScanDescriptor();

            _portName = portName;
            StartCaptureLoop();
        }

        private void StopDevice()
        {
            StopCaptureLoop();
            if (_port != null)
            {
                try { SendCommand(0x25); } catch { }   // STOP
                try { _port.DtrEnable = true; } catch { } // motor off
                try { _port.Close(); } catch { }
                try { _port.Dispose(); } catch { }
                _port = null;
            }
            _portName = null;
            lock (_scanLock) { _latestScan = null; }
        }
        // HQ motor speed control (cmd 0xA8). 600rpm = 10Hz (default).
        // Valid range varies by unit, roughly 480-1200rpm (8-20Hz). Check via RoboStudio or getMotorInfo.
        private void SetMotorRpm(int rpm)
        {
            byte lo = (byte)(rpm & 0xFF);
            byte hi = (byte)((rpm >> 8) & 0xFF);
            byte[] pkt = { 0xA5, 0xA8, 0x02, lo, hi, 0x00 };
            byte cs = 0;
            for (int i = 0; i < 5; i++) cs ^= pkt[i];   // checksum = XOR of the first 5 bytes
            pkt[5] = cs;
            _port.Write(pkt, 0, 6);
        }
        private void SendCommand(byte cmd)
        {
            // Request packet: start flag 0xA5, then the command byte.
            _port.Write(new byte[] { 0xA5, cmd }, 0, 2);
        }

        private void ReadScanDescriptor()
        {
            // Response descriptor is 7 bytes beginning with 0xA5 0x5A.
            var d = ReadExactly(7);
            if (d[0] != 0xA5 || d[1] != 0x5A)
                throw new InvalidOperationException("Bad scan response descriptor (wrong baud or port?).");
        }

        // ─── Capture loop ──────────────────────────────────────────────────

        private void StartCaptureLoop()
        {
            if (_captureThread != null) return;
            _captureCancellation = new CancellationTokenSource();
            var token = _captureCancellation.Token;
            _captureThread = new Thread(() => CaptureLoop(token))
            {
                IsBackground = true,
                Name = "RPLIDAR Capture Loop",
            };
            _captureThread.Start();
        }

        private void StopCaptureLoop()
        {
            _captureCancellation?.Cancel();
            _captureThread?.Join(500);
            _captureThread = null;
            _captureCancellation?.Dispose();
            _captureCancellation = null;
        }

        /// <summary>
        /// Reads the 5-byte node stream, accumulating points until the Start
        /// flag marks a new rotation, then publishes the completed rotation as
        /// the latest scan (latest-wins; no disposal needed since LidarFrame is
        /// immutable managed memory).
        /// </summary>
        private void CaptureLoop(CancellationToken token)
        {
            var current = new List<LidarPoint>(720);
            var node = new byte[5];

            while (!token.IsCancellationRequested)
            {
                try
                {
                    ReadNode(node, token);
                }
                catch (TimeoutException) { continue; }
                catch { break; }   // port closed / disposed → exit cleanly

                byte b0 = node[0], b1 = node[1], b2 = node[2], b3 = node[3], b4 = node[4];

                bool start = (b0 & 0x1) != 0;
                double angle = ((b1 >> 1) | (b2 << 7)) / 64.0;        // degrees
                double distMm = (b3 | (b4 << 8)) / 4.0;               // millimeters
                int quality = b0 >> 2;

                if (start && current.Count > 0)
                {
                    var now = DateTime.UtcNow;
                    double hz = 0;
                    if (_lastRotUtc != DateTime.MinValue)
                    {
                        double dt = (now - _lastRotUtc).TotalSeconds;
                        if (dt > 0)
                        {
                            double inst = 1.0 / dt;
                            _hzEma = _hzEma == 0 ? inst : _hzEma * 0.8 + inst * 0.2;  // slight smoothing
                            hz = _hzEma;
                        }
                    }
                    _lastRotUtc = now;

                    var frame = new LidarFrame(current.ToArray(), hz);
                    lock (_scanLock) { _latestScan = frame; }
                    current = new List<LidarPoint>(720);
                }

                if (distMm > 0)
                    current.Add(new LidarPoint(angle, distMm, quality));
            }
        }

        public LidarFrame AcquireLatestScan()
        {
            lock (_scanLock) { return _latestScan; }
        }

        // ─── Serial read helpers ───────────────────────────────────────────

        /// <summary>
        /// Reads one valid 5-byte measurement node, resynchronizing to a node
        /// boundary if the two parity invariants don't hold (handles dropped bytes).
        /// </summary>
        private void ReadNode(byte[] node, CancellationToken token)
        {
            ReadExactly(node, 5, token);
            while (!IsValidNode(node))
            {
                // Misaligned: drop the first byte, shift left, read one more.
                node[0] = node[1]; node[1] = node[2];
                node[2] = node[3]; node[3] = node[4];
                ReadExactly(node, 1, token, offset: 4);
            }
        }

        private static bool IsValidNode(byte[] n)
        {
            bool s = (n[0] & 0x1) != 0;
            bool sInv = (n[0] & 0x2) != 0;
            bool check = (n[1] & 0x1) != 0;   // must be 1
            return (s != sInv) && check;       // S and !S must differ; check bit set
        }

        private byte[] ReadExactly(int count)
        {
            var buf = new byte[count];
            ReadExactly(buf, count, CancellationToken.None);
            return buf;
        }

        private void ReadExactly(byte[] buf, int count, CancellationToken token, int offset = 0)
        {
            int read = 0;
            while (read < count)
            {
                if (token.IsCancellationRequested)
                    throw new OperationCanceledException();
                read += _port.Read(buf, offset + read, count - read);
            }
        }

        // ─── Cleanup when component removed from canvas ────────────────────
        public override void RemovedFromDocument(GH_Document document)
        {
            base.RemovedFromDocument(document);
            StopDevice();
        }


        public override GH_Exposure Exposure => GH_Exposure.primary;
        private static System.Drawing.Bitmap _icon;
        protected override System.Drawing.Bitmap Icon => _icon ?? (_icon = IconLoader.Load("RPLidar.png"));

        // Unique GUID — keep stable so old .ghx files keep resolving this component.
        public override Guid ComponentGuid => new Guid("3f1c8a42-6d7e-4b9a-9c21-5e8f0a1b2c3d");
    }
}