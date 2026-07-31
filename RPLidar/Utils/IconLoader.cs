using System;
using System.Drawing;
using System.Linq;
using System.Reflection;

namespace RPLidar      // RPLidar (not RPLidar.Utils) - parent namespace so it is visible everywhere
{
    internal static class IconLoader
    {
        // Load an embedded PNG by file name regardless of folder location (e.g. "outliner.png")
        public static Bitmap Load(string fileName)
        {
            var asm = Assembly.GetExecutingAssembly();
            var name = asm.GetManifestResourceNames()
                          .FirstOrDefault(n => n.EndsWith(fileName, StringComparison.OrdinalIgnoreCase));
            if (name == null) return null;
            using (var s = asm.GetManifestResourceStream(name))
                return s == null ? null : new Bitmap(s);
        }
    }
}