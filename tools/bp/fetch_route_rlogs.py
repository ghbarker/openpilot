#!/usr/bin/env python3
"""BluePilot: download every rlog of a route from comma in one shot.

One-time setup (browser login, stores a token in ~/.comma/auth.json):
  python3 tools/lib/auth.py

Then:
  python3 tools/bp/fetch_route_rlogs.py bfef784d32f5351d/00000006--319e078ab5 [out_dir]

Files land as <dongle>_<route>--<seg>--rlog.zst — the exact naming
tools/bp/angle_autocal_analyze.py discovers. Already-downloaded segments are
skipped, so rerunning after more segments upload just fills the gaps.
Default out_dir is the Windows Downloads folder when running under WSL.
"""
import os
import sys
import urllib.request

from openpilot.tools.lib.route import Route


def default_out_dir() -> str:
  wsl_downloads = "/mnt/c/Users/hunth/Downloads"
  if os.path.isdir(wsl_downloads):
    return wsl_downloads
  return os.path.expanduser("~/Downloads")


def main():
  if len(sys.argv) < 2:
    print(__doc__)
    sys.exit(1)
  route_name = sys.argv[1].replace("/", "|")
  out_dir = sys.argv[2] if len(sys.argv) > 2 else default_out_dir()
  os.makedirs(out_dir, exist_ok=True)

  route = Route(route_name)
  paths = route.log_paths()
  dongle, _, drive = route_name.partition("|")
  total = len(paths)
  missing = sum(1 for p in paths if p is None)
  print(f"{route_name}: {total} segments, {total - missing} uploaded, {missing} not yet uploaded")

  for seg, url in enumerate(paths):
    if url is None:
      continue  # device hasn't uploaded this segment yet
    ext = ".zst" if ".zst" in url else (".bz2" if ".bz2" in url else "")
    out = os.path.join(out_dir, f"{dongle}_{drive}--{seg}--rlog{ext}")
    if os.path.exists(out) and os.path.getsize(out) > 0:
      print(f"  seg {seg:>3}: already here, skipping")
      continue
    print(f"  seg {seg:>3}: downloading...", end="", flush=True)
    try:
      urllib.request.urlretrieve(url, out)
      print(f" {os.path.getsize(out) // 1024} KB")
    except Exception as exc:
      print(f" FAILED ({exc})")
      if os.path.exists(out):
        os.remove(out)

  print(f"done -> {out_dir}")


if __name__ == "__main__":
  main()
