# wslg-window-dump.ps1
# Dump ALL visible top-level windows with owner process, rect, minimized state.
# Why: WSLg renders every Linux GUI app window inside the single Windows process `msrdc.exe`,
# so `Get-Process | Where MainWindowTitle` can NOT find e.g. an `rviz` window.
# This enumerates msrdc's windows so you can prove whether the window exists, is minimized,
# or is off-screen / covered.
#
# IMPORTANT: keep this file ASCII/English only. PowerShell 5.1 reads .ps1 as ANSI;
# UTF-8 (no BOM) non-ASCII makes it fail with 'The string is missing the terminator'.
#
# Usage:  powershell -ExecutionPolicy Bypass -File .\wslg-window-dump.ps1

Add-Type @"
using System;
using System.Runtime.InteropServices;
using System.Text;
public class WslgWin {
  public delegate bool EnumWindowsProc(IntPtr hWnd, IntPtr lParam);
  [DllImport("user32.dll")] static extern bool EnumWindows(EnumWindowsProc cb, IntPtr lParam);
  [DllImport("user32.dll")] static extern int GetWindowText(IntPtr hWnd, StringBuilder s, int n);
  [DllImport("user32.dll")] static extern bool IsWindowVisible(IntPtr hWnd);
  [DllImport("user32.dll")] static extern bool IsIconic(IntPtr hWnd);
  [DllImport("user32.dll")] static extern uint GetWindowThreadProcessId(IntPtr hWnd, out uint pid);
  [DllImport("user32.dll")] static extern bool GetWindowRect(IntPtr hWnd, out RECT r);
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
}
"@

Add-Type -AssemblyName System.Windows.Forms -ErrorAction SilentlyContinue
$b = [System.Windows.Forms.Screen]::PrimaryScreen.Bounds
Write-Output ("=== Primary screen: " + $b.Width + "x" + $b.Height + " ===")
Write-Output "=== All VISIBLE top-level windows ==="
[WslgWin]::DumpAll()

Write-Output ""
Write-Output "=== msrdc (WSLg compositor) present? ==="
$p = Get-Process msrdc -ErrorAction SilentlyContinue
if ($p) { Write-Output ("  msrdc pid=" + $p.Id + " -> WSLg compositor is ALIVE") }
else    { Write-Output "  msrdc NOT running -> WSLg compositor is down" }
