using System;
using System.Collections.Generic;
using Grasshopper.Kernel;
using Rhino.Geometry;


namespace RPLidar.Components
{
    /// <summary>
    /// Pulls the latest completed rotation from a live RPLIDAR device handle
    /// and outputs it as Rhino geometry. Drive it with a Grasshopper Timer
    /// component (right-click → attach Timer, e.g. 50 ms) so it re-solves and
    /// keeps fetching the newest scan.
    /// </summary>
    public class LidarScanComponent : GH_Component
    {
        public LidarScanComponent()
          : base("RPLIDAR Scan", "Scan",
                 "Reads the latest 360-degree scan from an RPLIDAR device as points.",
                 "Appendage", "RPLiDAR")
        {
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddGenericParameter(
                "Lidar", "Dev",
                "Live RPLIDAR device handle from the RPLIDAR Device component.",
                GH_ParamAccess.item);

            pManager.AddNumberParameter(
                "Scale", "S",
                "Scale factor from millimeters to model units (e.g. 0.001 for meters).",
                GH_ParamAccess.item, 1.0);
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddPointParameter("Points", "P", "Scan points on the world XY plane.", GH_ParamAccess.list);
            pManager.AddNumberParameter("Distances", "D", "Distances (scaled).", GH_ParamAccess.list);
            pManager.AddNumberParameter("Angles", "A", "Angles in degrees.", GH_ParamAccess.list);
            pManager.AddIntegerParameter("Count", "N", "Number of valid points.", GH_ParamAccess.item);
            pManager.AddNumberParameter("ScanHz", "Hz", "Measured rotation frequency.", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            GH_LidarDevice handle = null;
            double scale = 1.0;

            if (!DA.GetData(0, ref handle) || handle == null || !handle.IsValid)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "No live device connected.");
                return;
            }
            DA.GetData(1, ref scale);

            LidarFrame frame = handle.AcquireLatestScan();
            if (frame == null || frame.Count == 0)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Remark, "Waiting for first full rotation...");
                return;
            }

            var pts = new List<Point3d>(frame.Count);
            var dists = new List<double>(frame.Count);
            var angles = new List<double>(frame.Count);

            foreach (var p in frame.Points)
            {
                double r = p.DistanceMm * scale;
                double rad = p.AngleDeg * Math.PI / 180.0- Math.PI / 2;
                // 0 degrees along +X, increasing CCW. Flip sign on Y if your unit spins the other way.
                pts.Add(new Point3d(r * Math.Cos(rad), -r * Math.Sin(rad), 0.0));
                dists.Add(r);
                angles.Add(p.AngleDeg);
            }

            DA.SetDataList(0, pts);
            DA.SetDataList(1, dists);
            DA.SetDataList(2, angles);
            DA.SetData(3, frame.Count);
            DA.SetData(4, frame.ScanHz);
        }


        public override GH_Exposure Exposure => GH_Exposure.primary;
        private static System.Drawing.Bitmap _icon;
        protected override System.Drawing.Bitmap Icon => _icon ?? (_icon = IconLoader.Load("DottedCircle48.png"));

        public override Guid ComponentGuid => new Guid("7a2b9e15-4c83-4f6d-b1a0-2d9c6e3f4a5b");
    }
}