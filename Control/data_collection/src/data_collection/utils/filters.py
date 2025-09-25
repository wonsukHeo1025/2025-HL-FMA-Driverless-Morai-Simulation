#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Signal filtering utilities."""

import numpy as np
from typing import Union, Optional


class LowPassFilter:
    """Simple low-pass filter implementation."""
    
    def __init__(self, alpha: float = 0.7):
        """Initialize filter.
        
        Args:
            alpha: Filter coefficient (0 < alpha < 1)
                   Higher values = less filtering
        """
        if not 0 < alpha <= 1:
            raise ValueError(f"Alpha must be in (0, 1], got {alpha}")
        
        self.alpha = alpha
        self.value = None
    
    def update(self, new_value: Union[float, np.ndarray]) -> Union[float, np.ndarray]:
        """Update filter with new value.
        
        Args:
            new_value: New measurement
            
        Returns:
            Filtered value
        """
        if self.value is None:
            self.value = new_value
        else:
            self.value = self.alpha * new_value + (1 - self.alpha) * self.value
        
        return self.value
    
    def reset(self, initial_value: Optional[Union[float, np.ndarray]] = None):
        """Reset filter state.
        
        Args:
            initial_value: Optional initial value
        """
        self.value = initial_value
    
    def get_value(self) -> Optional[Union[float, np.ndarray]]:
        """Get current filtered value.
        
        Returns:
            Current value or None if not initialized
        """
        return self.value


class AdaptiveBiasEstimator:
    """Adaptive bias estimation for IMU or other sensors."""
    
    def __init__(self, alpha: float = 0.99):
        """Initialize bias estimator.
        
        Args:
            alpha: Adaptation rate (closer to 1 = slower adaptation)
        """
        self.alpha = alpha
        self.bias = np.zeros(3)
    
    def update(self, measurement: np.ndarray, 
               is_stationary: bool = False) -> np.ndarray:
        """Update bias estimate.
        
        Args:
            measurement: Current sensor measurement
            is_stationary: True if system is known to be stationary
            
        Returns:
            Current bias estimate
        """
        if is_stationary:
            # Update bias when stationary
            self.bias = self.alpha * self.bias + (1 - self.alpha) * measurement
        
        return self.bias
    
    def correct(self, measurement: np.ndarray) -> np.ndarray:
        """Apply bias correction to measurement.
        
        Args:
            measurement: Raw measurement
            
        Returns:
            Bias-corrected measurement
        """
        return measurement - self.bias
    
    def reset(self):
        """Reset bias estimate to zero."""
        self.bias = np.zeros_like(self.bias)