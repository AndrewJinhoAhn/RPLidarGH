using System;
using System.Collections.Generic;
using System.Linq;
using Rhino.Geometry;
using Grasshopper;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;

namespace RPLidar.Components
{
    /// <summary>
    /// Indoor floor LiDAR point cloud -> wall outline polyline.
    /// 1) Sequential RANSAC wall detection (distance/incidence-adaptive completeness gate)
    /// 2) Merge near-parallel, closely-spaced duplicate walls by refitting combined points
    /// 3) Greedy sequential endpoint assembly -> 4) angle-branched junctions -> 5) close loop
    /// </summary>
    public class OutlinerComponent : GH_Component
    {
        public OutlinerComponent()
          : base("Outliner", "Outliner",
                 "Detects wall lines from a LiDAR point cloud and assembles them into an outline polyline.",
                 "Appendage", "RPLiDAR")
        {
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddPointParameter("Points", "P", "Points on walls (LiDAR scan).", GH_ParamAccess.list);
            pManager.AddNumberParameter("Threshold", "T", "RANSAC inlier distance / point proximity (mm).", GH_ParamAccess.item, 50.0);
            pManager.AddIntegerParameter("MinInliers", "Min", "Minimum floor to reject degenerate fits (the real gate is the distance-adaptive completeness ratio).", GH_ParamAccess.item, 4);
            pManager.AddNumberParameter("MinLength", "L", "Minimum wall length (mm).", GH_ParamAccess.item, 200.0);
            pManager.AddNumberParameter("AngleDeg", "A", "Branch angle for corner-intersection vs straight-link (deg).", GH_ParamAccess.item, 30.0);
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddCurveParameter("Outline", "O", "Outer outline polyline.", GH_ParamAccess.item);
            pManager.AddLineParameter("Walls", "W", "Detected / merged wall lines.", GH_ParamAccess.list);
            pManager.AddPointParameter("Corners", "C", "Polyline vertices (debug).", GH_ParamAccess.list);
            pManager.AddPointParameter("WallPoints", "WP", "Points that formed each wall (branch = wall index).", GH_ParamAccess.tree);
            pManager.AddTextParameter("Info", "I", "Status string.", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            var points = new List<Point3d>();
            double threshold = 0, minLength = 0, angleDeg = 0;
            int minInliers = 0;

            DA.GetDataList(0, points);
            DA.GetData(1, ref threshold);
            DA.GetData(2, ref minInliers);
            DA.GetData(3, ref minLength);
            DA.GetData(4, ref angleDeg);

            if (points == null || points.Count < 10)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Not enough points.");
                return;
            }

            double thr = threshold > 0 ? threshold : 50.0;
            int absFlr = minInliers > 0 ? minInliers : 4;
            double minLen = minLength > 0 ? minLength : 200.0;
            double aDeg = angleDeg > 0 ? angleDeg : 30.0;

            // 1-2) Detect walls + merge duplicate walls by combined points
            List<List<Point3d>> wallPts;
            List<Line> wallLines = DetectWalls(points, thr, absFlr, minLen, out wallPts);
            DA.SetDataList(1, wallLines);

            var ptTree = new DataTree<Point3d>();
            for (int i = 0; i < wallPts.Count; i++)
                foreach (Point3d p in wallPts[i]) ptTree.Add(p, new GH_Path(i));
            DA.SetDataTree(3, ptTree);

            if (wallLines.Count < 3)
            {
                DA.SetData(4, string.Format("walls={0} (insufficient)", wallLines.Count));
                return;
            }

            // 3-5) Endpoint matching -> angle-branched corners -> chain assembly
            List<Point3d> cornerList;
            Polyline pl = BuildOutline(wallLines, aDeg, out cornerList);
            DA.SetDataList(2, cornerList);

            if (pl != null)
            {
                DA.SetData(0, pl.ToPolylineCurve());
                DA.SetData(4, string.Format("walls={0} verts={1} angleDeg={2:F0} outline OK",
                                            wallLines.Count, cornerList.Count, aDeg));
            }
            else
            {
                DA.SetData(4, string.Format("walls={0} outline FAIL (chain assembly failed)", wallLines.Count));
            }
        }

        // ── Wall detection + duplicate-wall point merge ─────────────────────────────
        private List<Line> DetectWalls(List<Point3d> points, double thr, int absFloor, double minLen,
            out List<List<Point3d>> wallPts)
        {
            const double COMPLETENESS = 0.4;   // accept as wall if >= 40% of geometrically expected points (tunable)

            double dThetaDeg = EstimateDThetaDeg(points);   // estimate actual angular resolution from the cloud

            var rawLines = new List<Line>();
            var rawPts = new List<List<Point3d>>();

            var remaining = new List<Point3d>(points);
            var rnd = new Random(42);
            double maxGap = Math.Max(300.0, thr * 4);
            int maxLines = 40;

            while (remaining.Count > absFloor && rawLines.Count < maxLines)
            {
                int n = remaining.Count;
                int[] best = new int[n]; int bestCount = 0;
                int[] cur = new int[n];

                for (int it = 0; it < 600; it++)
                {
                    int i0 = rnd.Next(n), i1 = rnd.Next(n);
                    if (i0 == i1) continue;
                    Point3d p0 = remaining[i0], p1 = remaining[i1];
                    if (p0.DistanceTo(p1) < thr * 2) continue;

                    int cc = 0;
                    for (int j = 0; j < n; j++)
                        if (PointLineDist2D(remaining[j], p0, p1) < thr) cur[cc++] = j;

                    if (cc > bestCount) { bestCount = cc; Array.Copy(cur, best, cc); }
                }
                if (bestCount < absFloor) break;

                var inPts = new List<Point3d>(bestCount);
                for (int i = 0; i < bestCount; i++) inPts.Add(remaining[best[i]]);

                Line f0 = FitLine2D(inPts);
                Vector3d d0 = f0.Direction; d0.Unitize();
                Point3d o0 = f0.From;
                var proj = new List<(Point3d pt, double t)>(inPts.Count);
                foreach (var p in inPts)
                    proj.Add((p, new Vector3d(p.X - o0.X, p.Y - o0.Y, 0) * d0));
                proj.Sort((a, b) => a.t.CompareTo(b.t));

                // Gap-based longest contiguous run (separates collinear-but-disjoint segments)
                int cs = 0, bs = 0, bl = 1;
                for (int i = 1; i < proj.Count; i++)
                {
                    if (proj[i].t - proj[i - 1].t > maxGap)
                    { if (i - cs > bl) { bl = i - cs; bs = cs; } cs = i; }
                }
                if (proj.Count - cs > bl) { bl = proj.Count - cs; bs = cs; }

                var cluster = new List<Point3d>(bl);
                for (int i = bs; i < bs + bl; i++) cluster.Add(proj[i].pt);

                RemoveClosest(remaining, cluster);

                if (cluster.Count < absFloor) continue;          // reject degenerate

                Line wall = FitFromPoints(cluster);

                // Distance/incidence-adaptive accept: observed vs geometrically-expected point count
                double bA = Math.Atan2(wall.From.Y, wall.From.X);
                double bB = Math.Atan2(wall.To.Y, wall.To.X);
                double subtendedDeg = AngleDiffDeg(bA, bB);                 // segment angle subtended at the sensor (origin)
                double expected = dThetaDeg > 1e-9 ? subtendedDeg / dThetaDeg : cluster.Count;
                double needed = Math.Max(absFloor, COMPLETENESS * expected);

                if (cluster.Count >= needed && wall.Length >= minLen)
                {
                    rawLines.Add(wall);
                    rawPts.Add(cluster);
                }
            }

            return MergeByPoints(rawLines, rawPts, 30.0, thr * 2, out wallPts);
        }

        // ── Merge duplicate walls by combining their point sets ─────────────────────
        private List<Line> MergeByPoints(
            List<Line> lines, List<List<Point3d>> pts, double angTolDeg, double distTol,
            out List<List<Point3d>> outPts)
        {
            double cosTol = Math.Cos(angTolDeg * Math.PI / 180.0);
            int n = lines.Count;
            bool[] used = new bool[n];
            var res = new List<Line>();
            outPts = new List<List<Point3d>>();

            for (int i = 0; i < n; i++)
            {
                if (used[i]) continue;
                used[i] = true;

                var group = new List<Point3d>(pts[i]);
                Line refLine = lines[i];

                bool grew = true;
                while (grew)
                {
                    grew = false;
                    Vector3d dr = refLine.Direction; dr.Unitize();
                    LineCurve cr = new LineCurve(refLine);
                    for (int j = 0; j < n; j++)
                    {
                        if (used[j]) continue;
                        Vector3d dj = lines[j].Direction; dj.Unitize();
                        if (Math.Abs(dr * dj) < cosTol) continue;        // direction

                        Point3d pa, pb;
                        if (!cr.ClosestPoints(new LineCurve(lines[j]), out pa, out pb)) continue;
                        if (pa.DistanceTo(pb) > distTol) continue;       // closest distance

                        group.AddRange(pts[j]);
                        used[j] = true;
                        refLine = FitFromPoints(group);
                        cr = new LineCurve(refLine);
                        grew = true;
                        break;
                    }
                }
                res.Add(refLine);
                outPts.Add(group);
            }
            return res;
        }

        // ── Greedy sequential assembly -> angle-branched junctions -> close ─────────
        private Polyline BuildOutline(List<Line> walls, double angleDeg, out List<Point3d> cornerList)
        {
            cornerList = new List<Point3d>();
            int n = walls.Count;
            if (n < 3) return null;

            double cosThr = Math.Cos(angleDeg * Math.PI / 180.0);

            // 2N endpoints: ep[2i]=From, ep[2i+1]=To
            var ep = new Point3d[2 * n];
            for (int i = 0; i < n; i++) { ep[2 * i] = walls[i].From; ep[2 * i + 1] = walls[i].To; }

            // Start at the longest wall, exit from its To end
            int start = 0; double maxLen = 0;
            for (int i = 0; i < n; i++) if (walls[i].Length > maxLen) { maxLen = walls[i].Length; start = i; }

            var used = new bool[n];
            var loop = new List<Point3d>();
            int cur = start, exitEnd = 1;
            used[cur] = true;

            for (int step = 1; step < n; step++)
            {
                int k = 2 * cur + exitEnd;                 // current wall's exit endpoint

                // Nearest endpoint among unused walls
                int bestM = -1; double bd = double.MaxValue;
                for (int j = 0; j < n; j++)
                {
                    if (used[j]) continue;
                    double d0 = ep[k].DistanceTo(ep[2 * j]);
                    double d1 = ep[k].DistanceTo(ep[2 * j + 1]);
                    if (d0 < bd) { bd = d0; bestM = 2 * j; }
                    if (d1 < bd) { bd = d1; bestM = 2 * j + 1; }
                }
                if (bestM < 0) break;

                int nextWall = bestM / 2;
                int entryEnd = bestM % 2;

                AddJunction(loop, walls, ep, cur, k, nextWall, bestM, cosThr);

                cur = nextWall;
                exitEnd = 1 - entryEnd;                    // proceed to the opposite end of the wall we entered
                used[cur] = true;
            }

            // Close: last wall's exit -> start wall's unused (From) end
            int kLast = 2 * cur + exitEnd;
            int kClose = 2 * start + 0;
            AddJunction(loop, walls, ep, cur, kLast, start, kClose, cosThr);

            if (loop.Count < 3) return null;
            cornerList = new List<Point3d>(loop);
            loop.Add(loop[0]);
            return new Polyline(loop);
        }

        // Junction between two walls: branch by in-between angle -> intersection (1 pt) / straight link (2 pts)
        private void AddJunction(List<Point3d> loop, List<Line> walls, Point3d[] ep,
            int wallA, int kA, int wallB, int kB, double cosThr)
        {
            Vector3d di = walls[wallA].Direction; di.Unitize();
            Vector3d dj = walls[wallB].Direction; dj.Unitize();
            bool intersectCorner = Math.Abs(di * dj) <= cosThr;   // in-between angle >= angleDeg

            if (intersectCorner)
            {
                Point3d X = ExtendIntersect(walls[wallA], walls[wallB]);
                if (!X.IsValid)
                    X = new Point3d((ep[kA].X + ep[kB].X) * 0.5, (ep[kA].Y + ep[kB].Y) * 0.5, 0);
                loop.Add(X);              // true corner: single extended-intersection point
            }
            else
            {
                loop.Add(ep[kA]);         // nearly parallel: current wall's exit endpoint
                loop.Add(ep[kB]);         // + next wall's entry endpoint -> straight link between them
            }
        }

        private Line FitFromPoints(List<Point3d> pts)
        {
            Line f = FitLine2D(pts);
            Vector3d d = f.Direction; d.Unitize();
            Point3d o = f.From;
            double tMin = double.MaxValue, tMax = double.MinValue;
            foreach (var p in pts)
            {
                double t = new Vector3d(p.X - o.X, p.Y - o.Y, 0) * d;
                if (t < tMin) tMin = t; if (t > tMax) tMax = t;
            }
            return new Line(o + d * tMin, o + d * tMax);
        }

        // ── Helpers ─────────────────────────────────────────────────────────────────
        private Point3d ExtendIntersect(Line a, Line b)
        {
            double x1 = a.From.X, y1 = a.From.Y, x2 = a.To.X, y2 = a.To.Y;
            double x3 = b.From.X, y3 = b.From.Y, x4 = b.To.X, y4 = b.To.Y;
            double den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
            if (Math.Abs(den) < 1e-9) return Point3d.Unset;
            double t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / den;
            return new Point3d(x1 + t * (x2 - x1), y1 + t * (y2 - y1), 0);
        }

        // Remove the cluster's points from 'remaining' by greedy nearest match (handles duplicate coords)
        private void RemoveClosest(List<Point3d> remaining, List<Point3d> cluster)
        {
            var toRemove = new List<int>();
            var taken = new bool[remaining.Count];
            foreach (var c in cluster)
            {
                int best = -1; double bestD = double.MaxValue;
                for (int i = 0; i < remaining.Count; i++)
                {
                    if (taken[i]) continue;
                    double d = remaining[i].DistanceTo(c);
                    if (d < bestD) { bestD = d; best = i; }
                }
                if (best >= 0) { taken[best] = true; toRemove.Add(best); }
            }
            toRemove.Sort((a, b) => b.CompareTo(a));
            foreach (int idx in toRemove) remaining.RemoveAt(idx);
        }

        // Perpendicular distance from point p to the infinite line through a,b (2D)
        private double PointLineDist2D(Point3d p, Point3d a, Point3d b)
        {
            double dx = b.X - a.X, dy = b.Y - a.Y, len = Math.Sqrt(dx * dx + dy * dy);
            if (len < 1e-10) return p.DistanceTo(a);
            return Math.Abs((p.X - a.X) * dy - (p.Y - a.Y) * dx) / len;
        }

        // Total least-squares (PCA) line fit via the 2x2 scatter matrix eigenvector
        private Line FitLine2D(List<Point3d> pts)
        {
            double mx = pts.Average(p => p.X), my = pts.Average(p => p.Y);
            double sxx = 0, syy = 0, sxy = 0;
            foreach (Point3d p in pts)
            { double dx = p.X - mx, dy = p.Y - my; sxx += dx * dx; syy += dy * dy; sxy += dx * dy; }
            double diff = (sxx - syy) * 0.5;
            double lambda = (sxx + syy) * 0.5 + Math.Sqrt(diff * diff + sxy * sxy);
            Vector3d dir = new Vector3d(sxy, lambda - sxx, 0);
            if (dir.Length < 1e-10) dir = new Vector3d(1, 0, 0);
            dir.Unitize();
            return new Line(new Point3d(mx, my, 0), new Point3d(mx + dir.X, my + dir.Y, 0));
        }

        // Estimate actual angular resolution (dTheta) from the cloud via the median bearing gap
        // (robust to motor RPM and multi-frame accumulation)
        private double EstimateDThetaDeg(List<Point3d> pts)
        {
            int n = pts.Count;
            if (n < 2) return 0.72;
            var b = new double[n];
            for (int i = 0; i < n; i++) b[i] = Math.Atan2(pts[i].Y, pts[i].X) * 180.0 / Math.PI;
            Array.Sort(b);
            var gaps = new List<double>(n);
            for (int i = 1; i < n; i++)
            {
                double g = b[i] - b[i - 1];
                if (g > 0.05) gaps.Add(g);          // ignore sub-0.05 deg gaps as duplicates/noise (accumulation-safe)
            }
            if (gaps.Count == 0) return 0.72;
            gaps.Sort();
            double med = gaps[gaps.Count / 2];
            return Math.Min(5.0, Math.Max(0.1, med));   // clamp to a sane range
        }

        // Minimum angle between two bearings (rad in), returned in degrees [0,180]
        private double AngleDiffDeg(double aRad, double bRad)
        {
            double d = Math.Abs(aRad - bRad) * 180.0 / Math.PI;
            d %= 360.0;
            if (d > 180.0) d = 360.0 - d;
            return d;
        }

        public override GH_Exposure Exposure => GH_Exposure.primary;
        private static System.Drawing.Bitmap _icon;
        protected override System.Drawing.Bitmap Icon => _icon ?? (_icon = IconLoader.Load("FullCircle48.png"));

        public override Guid ComponentGuid => new Guid("b8e4d1a7-3c92-4f56-a1d8-7e2f9c046b3a");
    }
}