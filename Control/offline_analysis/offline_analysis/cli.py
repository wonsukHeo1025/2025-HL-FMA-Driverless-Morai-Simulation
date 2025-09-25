"""Command-line interface for offline analysis."""

import argparse
from pathlib import Path
from datetime import datetime
import numpy as np
import matplotlib.pyplot as plt
import os

from .loader import load_csv_files
from .preprocess import preprocess_dataframe
from .estimator import fit_parameters

RESULT_DIR = Path(__file__).resolve().parent.parent / 'results'


def save_plot(v_true, v_pred, title: str):
    RESULT_DIR.mkdir(exist_ok=True)
    ts = datetime.now().strftime('%Y%m%d_%H%M%S')
    filename = RESULT_DIR / f'fit_{ts}.png'

    plt.figure(figsize=(10,4))
    plt.plot(v_true, label='True', linewidth=1.2, alpha=0.8)
    plt.plot(v_pred, label='Predicted', linewidth=1.2, alpha=0.8)
    plt.title(title + ' (Time-ordered validation)')
    plt.xlabel('Time-ordered sample index')
    plt.ylabel('Velocity [m/s]')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(filename, dpi=150)
    print(f'[INFO] Plot saved to {filename}')


def main():
    parser = argparse.ArgumentParser(description='Offline CSV analysis - Full dataset training')
    parser.add_argument('paths', nargs='*', help='CSV files or directories')
    parser.add_argument('--speed-limit', type=float, default=50.0, help='Speed limit km/h')
    parser.add_argument('--segment-size', type=int, default=500, help='Validation segment size')
    parser.add_argument('--segment-start', type=int, default=None, help='Start index for validation segment')
    
    # Parameter save/load
    parser.add_argument('--save-params', type=str, help='Save fitted parameters to .npz file')
    parser.add_argument('--load-params', type=str, help='Load parameters from .npz file')
    
    # Modes
    parser.add_argument('--fit-only', action='store_true', help='Only fit parameters, no validation')
    parser.add_argument('--validate-only', action='store_true', help='Only validate, requires --load-params')
    
    args = parser.parse_args()
    
    # Validate arguments
    if args.validate_only and not args.load_params:
        parser.error("--validate-only requires --load-params")
    
    if args.fit_only and args.validate_only:
        parser.error("Cannot use both --fit-only and --validate-only")
    
    # Default paths if not provided
    if not args.paths and not args.validate_only:
        args.paths = ['/home/user1/catkin_ws/src/Control/data_collection/src/data_collection/data']
    
    # Load or fit parameters
    if args.load_params:
        # Load saved parameters
        data = np.load(args.load_params)
        params = data['params']
        A, B, d = params
        print(f'[INFO] Loaded params from {args.load_params}: A={A:.6f}, B={B:.6f}, d={d:.6f}')
        
        if 'total_samples' in data:
            print(f'[INFO] Original training samples: {data["total_samples"]}')
    else:
        # Load & preprocess
        df = load_csv_files(args.paths)
        y, M = preprocess_dataframe(df, speed_limit_kmph=args.speed_limit)
        
        print(f'[INFO] Total samples: {len(y)}')

        # Fit using ALL data (no train/val split)
        params = fit_parameters(M, y)
        A, B, d = params
        print(f'[INFO] Final params (ALL data): A={A:.6f}, B={B:.6f}, d={d:.6f}')
        
        # Save parameters if requested
        if args.save_params:
            np.savez(args.save_params, 
                     params=params,
                     total_samples=len(y),
                     speed_limit=args.speed_limit,
                     timestamp=datetime.now().isoformat())
            print(f'[INFO] Parameters saved to {args.save_params}')
    
    # Exit if fit-only mode
    if args.fit_only:
        return
    
    # Validation (requires data to be loaded)
    if args.validate_only:
        # Need to load data for validation
        if not args.paths:
            args.paths = ['/home/user1/catkin_ws/src/Control/data_collection/src/data_collection/data']
        df = load_csv_files(args.paths)
        y, M = preprocess_dataframe(df, speed_limit_kmph=args.speed_limit)
        print(f'[INFO] Validation data samples: {len(y)}')
    
    # Validation on a continuous segment
    if args.segment_start is None:
        # Default: use middle segment
        segment_start = len(y) // 2
    else:
        segment_start = args.segment_start
    
    segment_end = min(segment_start + args.segment_size, len(y))
    
    # Extract segment for validation
    M_seg = M[segment_start:segment_end]
    y_seg = y[segment_start:segment_end]
    
    v_series = y_seg
    u_series = M_seg[:, 1]
    v0 = M_seg[0, 0]
    
    v_pred = np.empty_like(v_series)
    v_pred[0] = v0
    for k in range(len(v_series)-1):
        v_pred[k+1] = A * v_pred[k] + B * u_series[k] + d

    rmse = np.sqrt(((v_series - v_pred) ** 2).mean())
    print(f'[INFO] Segment [{segment_start}:{segment_end}] RMSE: {rmse:.4f} m/s')

    save_plot(v_series, v_pred, f'Segment Validation (idx {segment_start}-{segment_end}, RMSE={rmse:.3f} m/s)')


if __name__ == '__main__':
    main()