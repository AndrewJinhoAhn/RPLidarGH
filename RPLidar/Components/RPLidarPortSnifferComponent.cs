using System;
using System.Collections.Generic;
using System.IO.Ports;
using System.Text;
using System.Threading;
using Grasshopper.Kernel;

namespace RPLidar.Components
{
    /// <summary>
    /// Scans serial ports and identifies connected RPLIDAR devices by probing each
    /// with GET_INFO (0xA5 0x50) across the known SLAMTEC baud rates. Model-agnostic
    /// (A / S / C series). Feed Port (and Baud) into the RPLIDAR Device component.
    /// Run this while the Device component is OFF, or the port will be busy.
    /// </summary>
    public class RPLidarPortSnifferComponent : GH_Component
    {
        // A1=115200, A2/A3/S1=256000, C1=460800, S2/S3=1000000
        private static readonly int[] CandidateBauds = { 460800, 256000, 115200, 1000000 };

        public RPLidarPortSnifferComponent()
          : base("RPLIDAR Port Sniffer", "PortSniff",
                 "Scans serial ports and identifies connected RPLIDAR devices (any model) via GET_INFO.",
                 "Appendage", "RPLiDAR")
        {
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddBooleanParameter("Refresh", "R",
                "Toggle to re-scan the serial ports (attach a Button/Toggle).",
                GH_ParamAccess.item, true);
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddTextParameter("Device", "D", "Identified RPLIDAR description (model / firmware / serial).", GH_ParamAccess.list);
            pManager.AddTextParameter("Port", "P", "Serial port name (e.g. COM3).", GH_ParamAccess.list);
            pManager.AddIntegerParameter("Baud", "B", "Baud rate that responded.", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            bool refresh = true;
            DA.GetData(0, ref refresh);   // value unused; just forces a re-solve when toggled

            var devices = new List<string>();
            var ports = new List<string>();
            var bauds = new List<int>();

            foreach (string portName in SerialPort.GetPortNames())
            {
                foreach (int baud in CandidateBauds)
                {
                    if (TryProbe(portName, baud, out string info))
                    {
                        devices.Add(info);
                        ports.Add(portName);
                        bauds.Add(baud);
                        break;   // port identified -> next port
                    }
                }
            }

            if (ports.Count == 0)
                AddRuntimeMessage(GH_RuntimeMessageLevel.Remark,
                    "No RPLIDAR found. Check cable/driver, and that the Device component is OFF (port free).");

            DA.SetDataList(0, devices);
            DA.SetDataList(1, ports);
            DA.SetDataList(2, bauds);
        }

        // Open at 'baud', send GET_INFO, parse the reply. Returns false on any failure.
        private static bool TryProbe(string portName, int baud, out string info)
        {
            info = null;
            SerialPort port = null;
            try
            {
                port = new SerialPort(portName, baud, Parity.None, 8, StopBits.One)
                {
                    ReadTimeout = 350,
                    WriteTimeout = 350,
                };
                port.Open();
                port.DtrEnable = false;                        // keep motor off while probing

                port.Write(new byte[] { 0xA5, 0x25 }, 0, 2);   // STOP -> clean idle
                Thread.Sleep(20);
                port.DiscardInBuffer();

                port.Write(new byte[] { 0xA5, 0x50 }, 0, 2);   // GET_INFO

                // 7-byte response descriptor: A5 5A 14 00 00 00 04
                byte[] desc = ReadExactly(port, 7);
                if (desc == null || desc[0] != 0xA5 || desc[1] != 0x5A || desc[6] != 0x04)
                    return false;

                // 20-byte info: model, fw_minor, fw_major, hardware, serial[16]
                byte[] d = ReadExactly(port, 20);
                if (d == null) return false;

                int model = d[0], fwMinor = d[1], fwMajor = d[2], hw = d[3];
                var sn = new StringBuilder();
                for (int i = 4; i < 20; i++) sn.Append(d[i].ToString("X2"));

                info = string.Format("RPLIDAR  model 0x{0:X2}  fw {1}.{2}  hw {3}  SN {4}",
                                     model, fwMajor, fwMinor, hw, sn.ToString());
                return true;
            }
            catch
            {
                return false;   // busy port, no response, wrong baud, etc.
            }
            finally
            {
                try { port?.Close(); port?.Dispose(); } catch { }
            }
        }

        // Read exactly n bytes, or null on timeout.
        private static byte[] ReadExactly(SerialPort port, int n)
        {
            var buf = new byte[n];
            int read = 0;
            try
            {
                while (read < n)
                    read += port.Read(buf, read, n - read);
            }
            catch (TimeoutException) { return null; }
            return buf;
        }

        public override GH_Exposure Exposure => GH_Exposure.primary;
        private static System.Drawing.Bitmap _icon;
        protected override System.Drawing.Bitmap Icon => _icon ?? (_icon = IconLoader.Load("RPLidar.png"));

        public override Guid ComponentGuid => new Guid("f4c1a9d2-7b63-4e85-9a1c-3d6e2f8b04a7");
    }
}