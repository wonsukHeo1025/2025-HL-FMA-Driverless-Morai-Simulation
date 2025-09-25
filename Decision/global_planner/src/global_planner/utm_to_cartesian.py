#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Convert ENU (map) path file to UTM (EPSG:32652) CSV.

Input : ENU txt (format: x y [z])
Output: CSV (columns: utm_e, utm_n, u)

Formula: UTM_e = x + eastOffset,  UTM_n = y + northOffset
"""

import argparse
import csv
from pathlib import Path

# Require explicit input/output via CLI; no personal absolute defaults
DEFAULT_IN  = None
DEFAULT_OUT = None
DEFAULT_EAST_OFFSET  = 313008.55819800857
DEFAULT_NORTH_OFFSET = 4161698.628368007

def parse_line(line: str):
    s = line.strip()
    if not s or s.startswith("#"):
        return None
    parts = s.split()
    try:
        x = float(parts[0])
        y = float(parts[1])
        z = float(parts[2]) if len(parts) >= 3 else 0.0
    except Exception:
        return None
    return x, y, z

def convert_to_utm(infile: str, outfile: str, east_offset: float, north_offset: float):
    in_path = Path(infile)
    out_path = Path(outfile)
    if not in_path.exists():
        raise FileNotFoundError(f"Input file not found: {in_path}")

    out_path.parent.mkdir(parents=True, exist_ok=True)

    count = 0
    with in_path.open("r", encoding="utf-8", errors="ignore") as f, \
         out_path.open("w", newline="", encoding="utf-8") as g:
        writer = csv.writer(g)
        writer.writerow(["utm_e", "utm_n", "u"])  # header
        for line in f:
            parsed = parse_line(line)
            if parsed is None:
                continue
            x, y, u = parsed
            utm_e = x + east_offset
            utm_n = y + north_offset
            writer.writerow([f"{utm_e:.6f}", f"{utm_n:.6f}", f"{u:.6f}"])
            count += 1

    print(f"[OK] Wrote {count} points → {out_path}")

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--in",  dest="infile",  default=DEFAULT_IN,  help="input ENU path txt (required)")
    ap.add_argument("--out", dest="outfile", default=DEFAULT_OUT, help="output UTM csv (required)")
    ap.add_argument("--east_offset",  type=float, default=DEFAULT_EAST_OFFSET,  help="eastOffset")
    ap.add_argument("--north_offset", type=float, default=DEFAULT_NORTH_OFFSET, help="northOffset")
    args = ap.parse_args()
    if not args.infile or not args.outfile:
        ap.print_help()
        raise SystemExit(2)
    convert_to_utm(args.infile, args.outfile, args.east_offset, args.north_offset)

if __name__ == "__main__":
    main()
