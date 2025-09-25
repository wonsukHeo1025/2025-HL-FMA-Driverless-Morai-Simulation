"""Preprocess raw dataframe for model fitting."""

from typing import Tuple
import pandas as pd
import numpy as np

SPEED_LIMIT_KMPH = 50.0

__all__ = [
    'preprocess_dataframe',
]


def preprocess_dataframe(df: pd.DataFrame, speed_limit_kmph: float = SPEED_LIMIT_KMPH, return_timestamps: bool = False) -> Tuple:
    """Clean and prepare dataframe for least-squares fitting.

    Steps
    -----
    1. Drop rows with NaNs or invalid flag.
    2. Compute control input u = accel_cmd − brake_cmd.
    3. Filter rows where |velocity| < `speed_limit_kmph`.
    4. Build (y, M) where y = v[1:], M = [v[:-1], u[:-1], 1].

    Returns
    -------
    Tuple[np.ndarray, np.ndarray] or Tuple[np.ndarray, np.ndarray, np.ndarray]
        y vector and M matrix ready for `numpy.linalg.lstsq`.
        If return_timestamps=True, also returns timestamps array.
    """
    # Ensure essential columns exist
    required_cols = {'accel_cmd', 'brake_cmd', 'true_velocity_x'}
    missing = required_cols - set(df.columns)
    if missing:
        raise KeyError(f'Missing required columns: {missing}')

    # Drop NaNs
    df = df.dropna(subset=list(required_cols))

    # Compute control input and speed km/h
    df = df.copy()
    df['u'] = df['accel_cmd'] - df['brake_cmd']
    df['speed_kmph'] = df['true_velocity_x'] * 3.6

    # Speed limit filter
    df = df[df['speed_kmph'].abs() < speed_limit_kmph]

    # Need at least 2 samples
    if len(df) < 2:
        raise ValueError('Not enough samples below speed limit for fitting')

    v = df['true_velocity_x'].to_numpy()
    u = df['u'].to_numpy()

    # Build y and M (shifted by 1)
    y = v[1:]
    M = np.column_stack((v[:-1], u[:-1], np.ones(len(v) - 1)))

    if return_timestamps and 'timestamp' in df.columns:
        timestamps = df['timestamp'].to_numpy()[1:]  # Align with y
        return y, M, timestamps
    
    return y, M