"""Multi-segment validation with single parameter estimation.

Usage: python -m offline_analysis.multi_run [--segments 10] [--segment-size 500] [--speed-limit 60]

Trains once on all data, then validates on multiple continuous segments.
"""

import argparse
from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from datetime import datetime

from .loader import load_csv_files
from .preprocess import preprocess_dataframe
from .estimator import fit_parameters

# Hard-coded CSV directory (as requested)
DATA_DIR = Path("/home/user1/catkin_ws/src/Control/data_collection/src/data_collection/data")
RESULT_DIR = Path(__file__).resolve().parent.parent / 'results'


def validate_segment(params: np.ndarray, M_seg: np.ndarray, y_seg: np.ndarray):
    """Validate on a continuous segment."""
    A, B, d = params
    v_series = y_seg
    u_series = M_seg[:, 1]
    
    v_pred = np.empty_like(v_series)
    v_pred[0] = M_seg[0, 0]
    for k in range(len(v_series) - 1):
        v_pred[k + 1] = A * v_pred[k] + B * u_series[k] + d
    
    rmse = np.sqrt(((v_series - v_pred) ** 2).mean())
    return v_pred, rmse


def save_multi_plot(segments_data, params):
    """Save a multi-panel plot with all validation segments."""
    RESULT_DIR.mkdir(exist_ok=True)
    ts = datetime.now().strftime('%Y%m%d_%H%M%S')
    filename = RESULT_DIR / f'multi_segment_{ts}.png'
    
    A, B, d = params
    n_segments = len(segments_data)
    cols = 2
    rows = (n_segments + 1) // 2
    
    fig, axes = plt.subplots(rows, cols, figsize=(14, 3*rows))
    axes = axes.flatten() if n_segments > 1 else [axes]
    
    for idx, (seg_range, v_true, v_pred, rmse) in enumerate(segments_data):
        ax = axes[idx]
        ax.plot(v_true, label='True', linewidth=1.0, alpha=0.8)
        ax.plot(v_pred, label='Predicted', linewidth=1.0, alpha=0.8)
        ax.set_title(f'Segment {seg_range} - RMSE: {rmse:.3f} m/s')
        ax.set_xlabel('Time step')
        ax.set_ylabel('Velocity [m/s]')
        ax.legend(loc='upper right', fontsize=8)
        ax.grid(True, alpha=0.3)
    
    # Hide unused subplots
    for idx in range(n_segments, len(axes)):
        axes[idx].set_visible(False)
    
    fig.suptitle(f'Multi-Segment Validation (A={A:.4f}, B={B:.4f}, d={d:.4f})', fontsize=12)
    plt.tight_layout()
    plt.savefig(filename, dpi=150)
    print(f"[INFO] Multi-segment plot saved to {filename}")


def main():
    parser = argparse.ArgumentParser(description="Multi-segment validation (single training)")
    parser.add_argument("--segments", type=int, default=10, help="Number of validation segments")
    parser.add_argument("--segment-size", type=int, default=500, help="Size of each segment")
    parser.add_argument("--speed-limit", type=float, default=60.0, help="Speed limit km/h")
    parser.add_argument("--load-params", type=str, help="Load parameters from .npz file")
    parser.add_argument("--save-params", type=str, help="Save fitted parameters to .npz file")
    args = parser.parse_args()

    # Load data
    print("[INFO] Loading data...")
    df = load_csv_files([str(DATA_DIR)])
    y, M = preprocess_dataframe(df, speed_limit_kmph=args.speed_limit)
    print(f"[INFO] Total samples: {len(y)}")
    
    # Load or fit parameters
    if args.load_params:
        # Load saved parameters
        data = np.load(args.load_params)
        params = data['params']
        A, B, d = params
        print(f"[INFO] Loaded parameters from {args.load_params}: A={A:.6f} B={B:.6f} d={d:.6f}")
        if 'total_samples' in data:
            print(f"[INFO] Original training samples: {data['total_samples']}")
    else:
        # Fit parameters
        print("[INFO] Fitting parameters...")
        params = fit_parameters(M, y)
        A, B, d = params
        print(f"[INFO] Fixed parameters: A={A:.6f} B={B:.6f} d={d:.6f}")
        
        # Save if requested
        if args.save_params:
            from datetime import datetime
            np.savez(args.save_params,
                     params=params,
                     total_samples=len(y),
                     speed_limit=args.speed_limit,
                     timestamp=datetime.now().isoformat())
            print(f"[INFO] Parameters saved to {args.save_params}")
    
    # Generate validation segments
    n_samples = len(y)
    segment_size = args.segment_size
    max_segments = min(args.segments, n_samples // segment_size)
    
    print(f"\n[INFO] Validating on {max_segments} segments of size {segment_size}")
    print("-" * 60)
    print(f"{'Segment':<15} {'Start':<10} {'End':<10} {'RMSE [m/s]':<12} {'Mean V [m/s]':<12}")
    print("-" * 60)
    
    rmse_list = []
    segments_data = []
    
    for i in range(max_segments):
        # Evenly distribute segments across the dataset
        start_idx = i * (n_samples - segment_size) // max(max_segments - 1, 1)
        end_idx = start_idx + segment_size
        
        M_seg = M[start_idx:end_idx]
        y_seg = y[start_idx:end_idx]
        
        v_pred, rmse = validate_segment(params, M_seg, y_seg)
        rmse_list.append(rmse)
        
        mean_v = y_seg.mean()
        seg_range = f"[{start_idx}:{end_idx}]"
        segments_data.append((seg_range, y_seg, v_pred, rmse))
        
        print(f"Segment {i:02d}     {start_idx:<10} {end_idx:<10} {rmse:<12.4f} {mean_v:<12.4f}")
    
    print("-" * 60)
    
    # Statistics
    mean_rmse = np.mean(rmse_list)
    std_rmse = np.std(rmse_list)
    print(f"\n=========== Summary ===========")
    print(f"Fixed Params : A={A:.6f}, B={B:.6f}, d={d:.6f}")
    print(f"RMSE Stats   : {mean_rmse:.4f} ± {std_rmse:.4f} m/s over {max_segments} segments")
    print(f"Min/Max RMSE : {min(rmse_list):.4f} / {max(rmse_list):.4f} m/s")
    print("================================")
    
    # Save multi-panel plot
    save_multi_plot(segments_data[:8], params)  # Limit to 8 panels for readability


if __name__ == "__main__":
    main()