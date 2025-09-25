"""Least-squares parameter estimation utilities."""

from typing import Tuple
import numpy as np
import math

__all__ = [
    'fit_parameters',
    'predict',
    'validate_model',
]


def fit_parameters(M: np.ndarray, y: np.ndarray) -> np.ndarray:
    """Solve least-squares M p ≈ y.
    
    Parameters
    ----------
    M : np.ndarray
        Design matrix [v[:-1], u[:-1], 1]
    y : np.ndarray
        Target vector v[1:]
        
    Returns
    -------
    np.ndarray
        Parameters [A, B, d]
    """
    p, *_ = np.linalg.lstsq(M, y, rcond=None)
    return p  # [A, B, d]


def predict(params: np.ndarray, v0: float, u: np.ndarray) -> np.ndarray:
    A, B, d = params
    v_pred = np.empty_like(u, dtype=float)
    v_pred[0] = v0
    for k in range(len(u)-1):
        v_pred[k+1] = A * v_pred[k] + B * u[k] + d
    return v_pred


def validate_model(params: np.ndarray, v_series: np.ndarray, u_series: np.ndarray) -> Tuple[float, float]:
    """Compute RMSE and R² on given series."""
    v_pred = predict(params, v_series[0], u_series)
    rmse = math.sqrt(((v_series - v_pred) ** 2).mean())
    ss_res = np.sum((v_series - v_pred) ** 2)
    ss_tot = np.sum((v_series - v_series.mean()) ** 2)
    r2 = 1 - ss_res / ss_tot if ss_tot > 0 else float('nan')
    return rmse, r2