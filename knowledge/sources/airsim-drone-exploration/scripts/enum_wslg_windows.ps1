# enum_wslg_windows.ps1
# ---------------------------------------------------------------------------
# List VISIBLE top-level windows (proc / rect / minimized), and OPTIONALLY
# resize/relocate a window by title substring via Win32 SetWindowPos.
#
# WHY: when the user says "the RViz window will not open / Alt+Tab cannot switch
#      to it", the answer is often that the window DOES exist -- it is just not
#      painted, or it is full-screen/off-screen. All WSLg (Linux GUI) app windows
#      are owned by the single Windows process `msrdc.exe` (the WSLg RDP
#      compositor), so `Get-Process | Where MainWindowTitle` CANNOT find them.
#      This script talks to Win32 directly instead.
#
# Run (list mode):
#   powershell -ExecutionPolicy Bypass -File enum_wslg_windows.ps1
# Run (resize mode) -- shrink the RViz window to 960x640 at (120,80):
#   powershell -ExecutionPolicy Bypass -File enum_wslg_windows.ps1 rviz 120 80 960 640
#
# How to read list output:
#   * "[WARN:COPY MODE] fast_livo2.rviz - RViz (ros1-twin)" => RViz EXISTS; the
#     COPY MODE prefix = WSLg software compositing (no GPU) -> paints badly.
#   * "ROS 1 End-of-Life is May 31st, 2025" => ROS Noetic EOL dialog, NOT an RViz error.
#   * minimized=True + rect=(-21333,-21333) is Windows' minimized sentinel.
#   * A window with size ~= screen size is just FULLSCREEN, not "broken" -- shrink
#     it with resize mode instead of concluding the app failed to open.
#
# NOTE: keep this file ASCII/English only. PowerShell 5.1 reads UTF-8-without-BOM
#       as ANSI, so Chinese gets mangled -> parse errors. Put Chinese in .sh (WSL).
#       Also: ONE Add-Type block only -- two blocks make the 2nd one get parsed
#       as PowerShell ("A 'using' statement must appear before any other...").
# ---------------------------------------------------------------------------

Add-Type @"
using System;
using System.Runtime.InteropServices;
using System.Text;
public class WslgWinEnum {
  public delegate bool EnumWindowsProc(IntPtr hWnd, IntPtr lParam);
  [DllImport("user32.dll")] static extern bool EnumWindows(EnumWindowsProc cb, IntPtr lParam);
  [DllImport("user32.dll")] static extern int  GetWindowText(IntPtr hWnd, StringBuilder s, int n);
  [DllImport("user32.dll")] static extern bool IsWindowVisible(IntPtr hWnd);
  [DllImport("user32.dll")] static extern bool IsIconic(IntPtr hWnd);
  [DllImport("user32.dll")] static extern uint GetWindowThreadProcessId(IntPtr hWnd, out uint pid);
  [DllImport("user32.dll")] static extern bool GetWindowRect(IntPtr hWnd, out RECT r);
  [DllImport("user32.dll")] static extern bool SetWindowPos(IntPtr h, IntPtr after, int x, int y, int cx, int cy, uint flags);
  public struct RECT { public int Left, Top, Right, Bottom; }

  public static string DumpAll() {
    var o = new StringBuilder();
    EnumWindows((h, l) => {
      uint pid; GetWindowThreadProcessId(h, out pid);
      var sb = new StringBuilder(512); GetWindowText(h, sb, 512);
      string t = sb.ToString();
      if (t.Length > 0 && IsWindowVisible(h)) {
        RECT r; GetWindowRect(h, out r);
        string pname = "?";
        try { pname = System.Diagnostics.Process.GetProcessById((int)pid).ProcessName; } catch {}
        o.AppendLine("  proc=" + pname + " pid=" + pid + " min=" + IsIconic(h) +
                     " rect=(" + r.Left + "," + r.Top + ")-(" + r.Right + "," + r.Bottom + ")" +
                     " size=" + (r.Right - r.Left) + "x" + (r.Bottom - r.Top) +
                     "  title=" + t);
      }
      return true;
    }, IntPtr.Zero);
    return o.ToString();
  }

  // Resize/relocate the first VISIBLE top-level window whose title contains `sub`
  // (case-insensitive). Returns NOTFOUND if none matches.
  public static string ResizeByTitle(string sub, int x, int y, int w, int h) {
    IntPtr target = IntPtr.Zero;
    EnumWindows((hw, l) => {
      var sb = new StringBuilder(512); GetWindowText(hw, sb, 512);
      if (IsWindowVisible(hw) && sb.ToString().ToLower().Contains(sub.ToLower())) { target = hw; return false; }
      return true;
    }, IntPtr.Zero);
    if (target == IntPtr.Zero) return "NOTFOUND (no visible window title contains '" + sub + "')";
    SetWindowPos(target, IntPtr.Zero, x, y, w, h, 0x0004 /* SWP_NOZORDER */);
    return "OK hwnd=0x" + target.ToString("X");
  }
}
"@

Add-Type -AssemblyName System.Windows.Forms -ErrorAction SilentlyContinue
$b = [System.Windows.Forms.Screen]::PrimaryScreen.Bounds
Write-Output ("=== Primary screen: " + $b.Width + "x" + $b.Height + " ===")

$argsList = @($args)
if ($argsList.Count -ge 5) {
  # resize mode: <title-substring> x y w h
  $sub = [string]$argsList[0]
  $x = [int]$argsList[1]; $y = [int]$argsList[2]
  $w = [int]$argsList[3]; $h = [int]$argsList[4]
  Write-Output ("Resize result: " + [WslgWinEnum]::ResizeByTitle($sub, $x, $y, $w, $h))
  Write-Output "=== after ==="
  [WslgWinEnum]::DumpAll()
} else {
  Write-Output "=== WSLg compositor (msrdc) ==="
  $m = Get-Process msrdc -ErrorAction SilentlyContinue
  if ($m) {
    Write-Output ("  msrdc pid=" + $m.Id)
    Get-CimInstance Win32_Process -Filter "Name='msrdc.exe'" -ErrorAction SilentlyContinue |
      Select-Object -ExpandProperty CommandLine
  } else {
    Write-Output "  msrdc NOT running -> WSLg display pipeline is down (no Linux GUI possible)"
  }
  Write-Output ""
  Write-Output "=== All VISIBLE top-level windows ==="
  [WslgWinEnum]::DumpAll()
}
