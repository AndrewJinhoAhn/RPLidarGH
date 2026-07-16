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
    /// Order-aware wall outline extractor - experimental alternative to Outliner.
    /// Exploits the angular ordering of a single LiDAR sweep (no RANSAC, deterministic):
    ///   1) split the ordered points into contiguous runs at range/angle gaps
    ///   2) Iterative End-Point Fit (IEPF) split each run into straight segments
    ///   3) merge adjacent near-collinear segments (split-and-merge)
    ///   4) assemble walls in scan order -> angle-branched junctions -> close loop
    /// Assumes the sensor sits at the world origin (same convention as Outliner).
    /// </summary>
    public class OutlinerSeqComponent : GH_Component
    {
        public OutlinerSeqComponent()
          : base("Outliner", "Outliner",
                 "Order-aware wall outline from an ordered single-sweep LiDAR scan (split-and-merge, no RANSAC).",
                 "Appendage", "RPLiDAR")
        {
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddPointParameter("Points", "P", "Scan points in scan (angular) order.", GH_ParamAccess.list);
            pManager.AddNumberParameter("Threshold", "T", "IEPF split / collinear-merge perpendicular tolerance (mm).", GH_ParamAccess.item, 50.0);
            pManager.AddNumberParameter("MinLength", "L", "Minimum wall length (mm).", GH_ParamAccess.item, 200.0);
            pManager.AddNumberParameter("AngleDeg", "A", "Branch angle for corner-intersection vs straight-link (deg).", GH_ParamAccess.item, 30.0);
            pManager.AddNumberParameter("GapFactor", "G", "Break a run when the gap between consecutive points exceeds this multiple of the expected (range-based) spacing.", GH_ParamAccess.item, 3.0);
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddCurveParameter("Outline", "O", "Outer outline polyline.", GH_ParamAccess.item);
            pManager.AddLineParameter("Walls", "W", "Detected wall lines.", GH_ParamAccess.list);
            pManager.AddPointParameter("Corners", "C", "Polyline vertices (debug).", GH_ParamAccess.list);
            pManager.AddPointParameter("WallPoints", "WP", "Points that formed each wall (branch = wall index).", GH_ParamAccess.tree);
            pManager.AddTextParameter("Info", "I", "Status string.", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            var points = new List<Point3d>();
            double threshold = 0, minLength = 0, angleDeg = 0, gapFactor = 0;

            DA.GetDataList(0, points);
            DA.GetData(1, ref threshold);
            DA.GetData(2, ref minLength);
            DA.GetData(3, ref angleDeg);
            DA.GetData(4, ref gapFactor);

            if (points == null || points.Count < 10)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Collection is too small.");
                return;
            }

            double thr = threshold > 0 ? threshold : 50.0;
            double minLen = minLength > 0 ? minLength : 100.0;
            double aDeg = angleDeg > 0 ? angleDeg : 30.0;
            double gFac = gapFactor > 0 ? gapFactor : 3.0;

            //split ordered points if the gap between consecutive points jumps too high (range-adaptive)
            List<List<Point3d>> runs = SplitRuns(points, gFac);

            //IEPF split each run into straight pieces -> fit each as a wall line
            var segLines = new List<Line>();
            var segPts = new List<List<Point3d>>();
            foreach (var run in runs)
            {
                if (run.Count < 2) continue;
                var pieces = new List<List<Point3d>>();
                IEPF(run, 0, run.Count - 1, thr, pieces);
                foreach (var piece in pieces)
                {
                    if (piece.Count < 2) continue;
                    segLines.Add(FitSegment(piece));
                    segPts.Add(piece);
                }
            }

            //merge adjacent near-collinear pieces (re-joins walls split by small gaps)
            MergeAdjacentCollinear(segLines, segPts, aDeg, thr);

            //drop too-short walls (after merge, so split halves get a chance to combine)
            for (int i = segLines.Count - 1; i >= 0; i--)
                if (segLines[i].Length < minLen) { segLines.RemoveAt(i); segPts.RemoveAt(i); }

            //second merge pass: with the short junk filtered out, collinear walls that
            //were separated by it are now adjacent and can finally join (kills spurious chamfers)
            MergeAdjacentCollinear(segLines, segPts, aDeg, thr);

            DA.SetDataList(1, segLines);

            var ptTree = new DataTree<Point3d>();
            for (int i = 0; i < segPts.Count; i++)
                foreach (Point3d p in segPts[i]) ptTree.Add(p, new GH_Path(i));
            DA.SetDataTree(3, ptTree);

            if (segLines.Count < 3)
            {
                DA.SetData(4, string.Format("runs={0} walls={1} (insufficient)", runs.Count, segLines.Count));
                return;
            }

            // 4) ordered assembly -> junctions -> close loop
            List<Point3d> corners;
            Polyline pl = AssembleOrdered(segLines, aDeg, out corners);
            DA.SetDataList(2, corners);

            if (pl != null)
            {
                DA.SetData(0, pl.ToPolylineCurve());
                DA.SetData(4, string.Format("runs={0} walls={1} verts={2} angleDeg={3:F0} (seq) OK",
                                            runs.Count, segLines.Count, corners.Count, aDeg));
            }
            else
            {
                DA.SetData(4, string.Format("runs={0} walls={1} outline FAIL", runs.Count, segLines.Count));
            }
        }

        // ?? 1) Run splitting by range-adaptive gap ???????????????????????????
        private List<List<Point3d>> SplitRuns(List<Point3d> pts, double gapFactor)
        {
            int n = pts.Count;
            double dTheta = EstimateDThetaRad(pts);

            var runs = new List<List<Point3d>>();
            var cur = new List<Point3d> { pts[0] };
            for (int i = 1; i < n; i++)
            {
                double rA = pts[i - 1].DistanceTo(Point3d.Origin);
                double rB = pts[i].DistanceTo(Point3d.Origin);
                double expected = Math.Max(1.0, 0.5 * (rA + rB) * dTheta); //arc distance between consecutive points
                double gap = pts[i - 1].DistanceTo(pts[i]);
                if (gap > gapFactor * expected) { runs.Add(cur); cur = new List<Point3d>(); }
                cur.Add(pts[i]);
            }
            if (cur.Count > 0) runs.Add(cur);

            // wrap-around: join last and first run if continuous across 0/360
            if (runs.Count >= 2)
            {
                var first = runs[0];
                var last = runs[runs.Count - 1];
                Point3d lp = last[last.Count - 1], fp = first[0];
                double rA = lp.DistanceTo(Point3d.Origin);
                double rB = fp.DistanceTo(Point3d.Origin);
                double expected = Math.Max(1.0, 0.5 * (rA + rB) * dTheta);
                if (lp.DistanceTo(fp) <= gapFactor * expected)
                {
                    last.AddRange(first);
                    runs.RemoveAt(0);
                }
            }
            return runs;
        }

        //Iterative End-Point Fit(IEPF) split of a run into straight segments (recursive)
        private void IEPF(List<Point3d> run, int s, int e, double thr, List<List<Point3d>> outPieces)
        {
            if (e - s < 1) return;

            Point3d a = run[s], b = run[e];
            double maxD = -1; int mi = -1;
            for (int i = s + 1; i < e; i++)
            {
                double d = PointLineDist2D(run[i], a, b);
                if (d > maxD) { maxD = d; mi = i; }
            }

            if (maxD > thr && mi > s && mi < e)
            {
                IEPF(run, s, mi, thr, outPieces);
                IEPF(run, mi, e, thr, outPieces);
            }
            else
            {
                var piece = new List<Point3d>(e - s + 1);
                for (int i = s; i <= e; i++) piece.Add(run[i]);
                outPieces.Add(piece);
            }
        }

        // ?? 3) Merge adjacent collinear segments (same wall split by a gap) ???
        private void MergeAdjacentCollinear(List<Line> lines, List<List<Point3d>> pts, double angleDeg, double thr)
        {
            double cosTol = Math.Cos(angleDeg * Math.PI / 180.0);
            int i = 0;
            while (i < lines.Count - 1)
            {
                Vector3d di = lines[i].Direction; di.Unitize();
                Vector3d dj = lines[i + 1].Direction; dj.Unitize();
                bool sameDir = Math.Abs(di * dj) >= cosTol;

                // lateral offset of the next segment onto this line's axis (confirms same wall, not a parallel one)
                double offA = PointLineDist2D(lines[i + 1].From, lines[i].From, lines[i].To);
                double offB = PointLineDist2D(lines[i + 1].To, lines[i].From, lines[i].To);
                bool sameLine = offA < thr * 1.5 && offB < thr * 1.5;

                if (sameDir && sameLine)
                {
                    var merged = new List<Point3d>(pts[i]);
                    merged.AddRange(pts[i + 1]);
                    lines[i] = FitSegment(merged);
                    pts[i] = merged;
                    lines.RemoveAt(i + 1);
                    pts.RemoveAt(i + 1);
                    // stay on i to try merging the new combined segment with the next
                }
                else i++;
            }
        }

        // ?? 4) Ordered assembly ??????????????????????????????????????????????
        private Polyline AssembleOrdered(List<Line> walls, double angleDeg, out List<Point3d> corners)
        {
            corners = new List<Point3d>();
            int n = walls.Count;
            if (n < 3) return null;
            double cosThr = Math.Cos(angleDeg * Math.PI / 180.0);

            var loop = new List<Point3d>();
            for (int k = 0; k < n; k++)
            {
                Line a = walls[k];
                Line b = walls[(k + 1) % n];
                AddJunction(loop, a, a.To, b, b.From, cosThr);
            }
            if (loop.Count < 3) return null;
            corners = new List<Point3d>(loop);
            loop.Add(loop[0]);
            return new Polyline(loop);
        }

        private void AddJunction(List<Point3d> loop, Line a, Point3d exitA, Line b, Point3d entryB, double cosThr)
        {
            Vector3d di = a.Direction; di.Unitize();
            Vector3d dj = b.Direction; dj.Unitize();
            bool corner = Math.Abs(di * dj) <= cosThr;   // in-between angle >= angleDeg

            if (corner)
            {
                Point3d X = ExtendIntersect(a, b);
                if (!X.IsValid)
                    X = new Point3d((exitA.X + entryB.X) * 0.5, (exitA.Y + entryB.Y) * 0.5, 0);
                loop.Add(X);                 // true corner: single intersection vertex
            }
            else
            {
                loop.Add(exitA);             // nearly collinear: bridge with both endpoints
                loop.Add(entryB);
            }
        }

        // ?? Helpers ??????????????????????????????????????????????????????????
        // Fit a line to the points and orient From->To along scan order (first..last point).
        private Line FitSegment(List<Point3d> pts)
        {
            Line f = FitLine2D(pts);
            Vector3d d = f.Direction; d.Unitize();
            Point3d o = f.From;
            double tMin = double.MaxValue, tMax = double.MinValue;
            foreach (var p in pts)
            {
                double t = (p.X - o.X) * d.X + (p.Y - o.Y) * d.Y;
                if (t < tMin) tMin = t;
                if (t > tMax) tMax = t;
            }
            Point3d e0 = o + d * tMin, e1 = o + d * tMax;
            Point3d first = pts[0], last = pts[pts.Count - 1];
            if (first.DistanceTo(e0) + last.DistanceTo(e1) <= first.DistanceTo(e1) + last.DistanceTo(e0))
                return new Line(e0, e1);
            return new Line(e1, e0);
        }

        private Point3d ExtendIntersect(Line a, Line b)
        {
            double x1 = a.From.X, y1 = a.From.Y, x2 = a.To.X, y2 = a.To.Y;
            double x3 = b.From.X, y3 = b.From.Y, x4 = b.To.X, y4 = b.To.Y;
            double den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
            if (Math.Abs(den) < 1e-9) return Point3d.Unset;
            double t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / den;
            return new Point3d(x1 + t * (x2 - x1), y1 + t * (y2 - y1), 0);
        }

        private double PointLineDist2D(Point3d p, Point3d a, Point3d b)
        {
            double dx = b.X - a.X, dy = b.Y - a.Y, len = Math.Sqrt(dx * dx + dy * dy);
            if (len < 1e-10) return p.DistanceTo(a);
            return Math.Abs((p.X - a.X) * dy - (p.Y - a.Y) * dx) / len;
        }

        // Total least-squares (PCA) line fit via the 2x2 scatter-matrix eigenvector
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

        // Median angular step (radians), robust to RPM / accumulation
        private double EstimateDThetaRad(List<Point3d> pts)
        {
            int n = pts.Count;
            //assumes 500 pts per 360 deg sweep on a ideal condition, so ~0.72 deg per step (0.01257 rad) is a reasonable fallback
            double fallback = 0.72 * Math.PI / 180.0;
            if (n < 2) return fallback;
            var b = new double[n];
            for (int i = 0; i < n; i++) b[i] = Math.Atan2(pts[i].Y, pts[i].X);
            Array.Sort(b);
            var gaps = new List<double>(n);
            for (int i = 1; i < n; i++)
            {
                double g = b[i] - b[i - 1];
                if (g > 0.0009) gaps.Add(g);   // ignore < ~0.05 deg as duplicates/noise
            }
            if (gaps.Count == 0) return fallback;
            //minimize effect of outliers by taking the median gap
            gaps.Sort();
            double med = gaps[gaps.Count / 2];
            //return Math.Min(5.0 * Math.PI / 180.0, Math.Max(0.1 * Math.PI / 180.0, med));
            return Math.Clamp(med, 0.1 * Math.PI / 180.0, 5.0 * Math.PI / 180.0);
        }

        public override GH_Exposure Exposure => GH_Exposure.primary;
        private static System.Drawing.Bitmap _icon;
        protected override System.Drawing.Bitmap Icon => _icon ?? (_icon = IconLoader.Load("Outliner.png"));

        // New, unique GUID (distinct from the original Outliner) so both coexist.
        public override Guid ComponentGuid => new Guid("d2a7f3c1-9b56-4e08-a4d3-6c1e8f209b7a");
    }
}
