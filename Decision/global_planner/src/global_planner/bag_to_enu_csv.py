#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
bag_to_enu_csv.py
Convert MORAI /gps rosbag(s) to ENU(map/reference) CSV with columns: index,e,n,u

- Reads morai_msgs/GPSMessage (latitude, longitude, altitude, eastOffset, northOffset)
- Converts lat/lon to UTM (easting, northing) and subtracts MORAI offsets:
    e = UTM_E - eastOffset
    n = UTM_N - northOffset
- Outputs CSV compatible with global_planner (e,n[,u])

Requirements:
    pip install utm

Usage examples:
    # Single bag
    rosrun gps_global_planner bag_to_enu_csv.py --bag /path/drive.bag --out /path/lane_right.csv

    # Directory of bags (processed in name order)
    rosrun gps_global_planner bag_to_enu_csv.py --bag /path/bags_dir --out /path/lane_right.csv

    # Custom topic name
    rosrun gps_global_planner bag_to_enu_csv.py --bag drive.bag --out lane.csv --topic /gps_morai

    # Fallback for NavSatFix with fixed origin
    rosrun gps_global_planner bag_to_enu_csv.py --bag drive.bag --out lane.csv \
        --fallback-navsatfix-mode latlon_fixed --ref-lat 37.33 --ref-lon 127.89

In-file configuration (no CLI):
    # Edit the DEFAULTS below and simply run:
    rosrun gps_global_planner bag_to_enu_csv.py
"""

import os
import sys
import csv
import math
import argparse
from typing import List, Optional, Tuple

try:
    import rosbag
except Exception:
    print("[ERROR] rosbag import failed. Run inside ROS (Noetic) environment.")
    sys.exit(1)

try:
    import utm
except Exception:
    print("[ERROR] Python package 'utm' not found. Install with: pip install utm")
    sys.exit(1)


# =========================
# USER-EDITABLE DEFAULTS
# If you prefer not to use CLI args, set these and run the script without args.
# Leave strings empty to force CLI usage.
DEFAULTS = {
    # Path to a .bag or directory containing .bag files (empty → require CLI or env)
    'BAG_PATH': '',
    # Output CSV path (empty → require CLI or env)
    'OUT_CSV': '',
    # Optional explicit topic name (otherwise auto-detected)
    'TOPIC': None,                # e.g., '/gps'
    # NavSatFix fallback mode: 'utm_first' or 'latlon_fixed'
    'FALLBACK_NAVSATFIX_MODE': 'utm_first',
    # Reference lat/lon (only used with 'latlon_fixed')
    'REF_LAT': None,
    'REF_LON': None,
}


def list_bag_files(path: str) -> List[str]:
    """Return a sorted list of .bag files. Accepts a single .bag or a directory."""
    if os.path.isdir(path):
        bags = [os.path.join(path, f) for f in os.listdir(path) if f.endswith(".bag")]
        bags.sort()
        return bags
    if os.path.isfile(path) and path.endswith(".bag"):
        return [path]
    raise FileNotFoundError(f"Invalid bag path: {path}")


def autodetect_gps_topic(bag: rosbag.Bag, preferred=("/gps",)) -> Optional[str]:
    """Pick a GPS-like topic from bag, preferring given names and MORAI GPS type."""
    info = bag.get_type_and_topic_info()
    topics_info = getattr(info, "topics", {})
    # Prefer explicit candidates
    for t in preferred:
        if t in topics_info:
            return t
    # Otherwise pick first MORAI GPS topic
    for name, ti in topics_info.items():
        if ti.msg_type == "morai_msgs/GPSMessage":
            return name
    # As a fallback, sensor_msgs/NavSatFix
    for name, ti in topics_info.items():
        if ti.msg_type == "sensor_msgs/NavSatFix":
            return name
    return None


def extract_enu_from_morai_gps(msg) -> Optional[Tuple[float, float, float]]:
    """Return (e, n, u) in meters if msg is MORAI GPS, else None."""
    try:
        if getattr(msg, "_type", "") == "morai_msgs/GPSMessage":
            lat = float(msg.latitude)
            lon = float(msg.longitude)
            alt = float(getattr(msg, "altitude", 0.0))
            east_offset = float(getattr(msg, "eastOffset", 0.0))
            north_offset = float(getattr(msg, "northOffset", 0.0))
            if not (math.isfinite(lat) and math.isfinite(lon)):
                return None
            utm_e, utm_n, _, _ = utm.from_latlon(lat, lon)
            e = utm_e - east_offset
            n = utm_n - north_offset
            return e, n, alt
    except Exception:
        pass
    return None


def extract_enu_from_navsatfix(msg, ref_mode: str, ref_lat: float, ref_lon: float,
                               ref_utm_e: float, ref_utm_n: float) -> Optional[Tuple[float, float, float]]:
    """
    Fallback for sensor_msgs/NavSatFix when MORAI offsets are not present.
    - ref_mode='utm_first': subtract first sample's UTM as origin
    - ref_mode='latlon_fixed': subtract fixed reference lat/lon UTM
    """
    try:
        if getattr(msg, "_type", "") != "sensor_msgs/NavSatFix":
            return None
        lat = float(msg.latitude)
        lon = float(msg.longitude)
        alt = float(getattr(msg, "altitude", 0.0))
        if not (math.isfinite(lat) and math.isfinite(lon)):
            return None
        utm_e, utm_n, _, _ = utm.from_latlon(lat, lon)
        if ref_mode == "utm_first":
            e = utm_e - ref_utm_e
            n = utm_n - ref_utm_n
        elif ref_mode == "latlon_fixed":
            base_e, base_n, _, _ = utm.from_latlon(ref_lat, ref_lon)
            e = utm_e - base_e
            n = utm_n - base_n
        else:
            return None
        return e, n, alt
    except Exception:
        return None


def convert_bags_to_csv(bag_files: List[str], csv_out: str, topic: Optional[str],
                        fallback_navsatfix_mode: str, ref_lat: Optional[float], ref_lon: Optional[float]) -> None:
    out_dir = os.path.dirname(csv_out)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)

    index = 1
    wrote_any = False
    with open(csv_out, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["index", "e", "n", "u"])

        for bag_path in bag_files:
            print(f"[INFO] Processing: {bag_path}")
            with rosbag.Bag(bag_path, "r") as bag:
                chosen_topic = topic or autodetect_gps_topic(bag)
                if not chosen_topic:
                    print(f"[WARN] No GPS-like topic found in {bag_path}. Skipping.")
                    continue

                # Prepare NavSatFix fallback reference if needed
                navsatfix_ref_e = None
                navsatfix_ref_n = None

                msg_count = 0
                for _, msg, _ in bag.read_messages(topics=[chosen_topic]):
                    msg_count += 1

                    # Prefer MORAI GPS direct path
                    enu = extract_enu_from_morai_gps(msg)
                    if enu is not None:
                        e, n, u = enu
                        writer.writerow([index, f"{e:.6f}", f"{n:.6f}", f"{u:.3f}"])
                        index += 1
                        wrote_any = True
                        continue

                    # Fallback: NavSatFix
                    if getattr(msg, "_type", "") == "sensor_msgs/NavSatFix":
                        # Establish reference if needed
                        if fallback_navsatfix_mode == "utm_first" and (navsatfix_ref_e is None or navsatfix_ref_n is None):
                            try:
                                lat0 = float(msg.latitude)
                                lon0 = float(msg.longitude)
                                navsatfix_ref_e, navsatfix_ref_n, _, _ = utm.from_latlon(lat0, lon0)
                                print(
                                    f"[INFO] NavSatFix origin set to first sample UTM: "
                                    f"({navsatfix_ref_e:.3f}, {navsatfix_ref_n:.3f})"
                                )
                            except Exception:
                                pass

                        mode = "latlon_fixed" if (
                            fallback_navsatfix_mode == "latlon_fixed" and ref_lat is not None and ref_lon is not None
                        ) else "utm_first"

                        enu2 = extract_enu_from_navsatfix(
                            msg, mode, ref_lat or 0.0, ref_lon or 0.0,
                            navsatfix_ref_e or 0.0, navsatfix_ref_n or 0.0
                        )
                        if enu2 is not None:
                            e, n, u = enu2
                            writer.writerow([index, f"{e:.6f}", f"{n:.6f}", f"{u:.3f}"])
                            index += 1
                            wrote_any = True

                print(f"[INFO] {bag_path}: messages read={msg_count}")

    if not wrote_any:
        print("[ERROR] No valid GPS samples written. Check topic name and message types.")
    else:
        print(f"[OK] CSV written: {csv_out}")


def main():
    ap = argparse.ArgumentParser(description="Convert MORAI /gps rosbag(s) to ENU CSV (index,e,n,u)")
    ap.add_argument("--bag", required=False, help="Path to a .bag or a directory containing .bag files (overrides DEFAULTS)")
    ap.add_argument("--out", required=False, help="Output CSV path (overrides DEFAULTS)")
    ap.add_argument("--topic", default=None, help="GPS topic name (default: auto-detect, prefer /gps)")
    ap.add_argument(
        "--fallback-navsatfix-mode",
        choices=["utm_first", "latlon_fixed"],
        default=None,
        help="When input is NavSatFix, choose origin: first sample UTM or fixed ref lat/lon",
    )
    ap.add_argument("--ref-lat", type=float, default=None, help="Reference latitude (used with --fallback-navsatfix-mode=latlon_fixed)")
    ap.add_argument("--ref-lon", type=float, default=None, help="Reference longitude (used with --fallback-navsatfix-mode=latlon_fixed)")
    args = ap.parse_args()

    # Resolve precedence: CLI > ENV > DEFAULTS
    bag_path = args.bag or os.environ.get('BAG_TO_ENU_BAG') or DEFAULTS.get('BAG_PATH')
    out_csv = args.out or os.environ.get('BAG_TO_ENU_OUT') or DEFAULTS.get('OUT_CSV')
    topic = args.topic if args.topic is not None else DEFAULTS.get('TOPIC')
    fb_mode = args.fallback_navsatfix_mode or DEFAULTS.get('FALLBACK_NAVSATFIX_MODE', 'utm_first')
    ref_lat = args.ref_lat if args.ref_lat is not None else DEFAULTS.get('REF_LAT')
    ref_lon = args.ref_lon if args.ref_lon is not None else DEFAULTS.get('REF_LON')

    if not bag_path or not out_csv:
        ap.print_help()
        print("\n[ERROR] Provide --bag/--out, or set env BAG_TO_ENU_BAG/BAG_TO_ENU_OUT.)")
        sys.exit(2)

    bags = list_bag_files(bag_path)
    convert_bags_to_csv(bags, out_csv, topic, fb_mode, ref_lat, ref_lon)


if __name__ == "__main__":
    main()
