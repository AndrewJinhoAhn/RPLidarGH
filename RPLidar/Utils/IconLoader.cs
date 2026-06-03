using System;
using System.Drawing;
using System.Linq;
using System.Reflection;

namespace RPLidar      // ★ RPLidar.Utils 아니라 RPLidar — 부모라 어디서든 보임
{
    internal static class IconLoader
    {
        // 폴더 위치 상관없이 파일명으로 임베디드 PNG 로드 (예: "outliner.png")
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