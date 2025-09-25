#!/usr/bin/env python3
"""
Verification script for P0 fixes:
1. Ackermann bias correction in Kv estimation
2. Step detection threshold adjustments
3. Import error fixes
4. Frequency range extension
"""

import os
import sys
import numpy as np
import pandas as pd
import logging

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def test_ackermann_correction():
    """Test Ackermann bias correction in Kv estimation."""
    logger.info("=" * 60)
    logger.info("TEST 1: Ackermann Bias Correction")
    logger.info("=" * 60)
    
    from dynamic_model_id import DynamicModelIdentifier
    
    # Create test data with known parameters
    # Simulate steady-state cornering data
    L = 2.875  # Wheelbase
    Kv_true = 0.025  # True understeer gradient
    
    # Generate synthetic data with proper structure
    speeds = [10, 15, 20]  # m/s
    ay_values = [2, 3, 4, 5]  # m/s²
    
    data_rows = []
    t = 0
    for v in speeds:
        for ay in ay_values:
            # Calculate steering angle with Ackermann + understeer
            R = v * v / ay  # Turning radius
            delta = L / R + Kv_true * ay  # Total steering angle
            omega = v / R  # Yaw rate
            
            # Create data row - need multiple samples for statistics
            for _ in range(100):  # Multiple samples for same condition
                data_rows.append({
                    'steering_angle': delta + np.random.normal(0, 0.0001),  # Small noise
                    'lateral_acceleration': ay + np.random.normal(0, 0.01),
                    'vehicle_speed': v + np.random.normal(0, 0.01),
                    'yaw_rate': omega + np.random.normal(0, 0.0001),
                    'is_steady_state': True,
                    'timestamp': t,
                    'scenario_time': t
                })
                t += 0.02  # 50 Hz sampling
    
    df = pd.DataFrame(data_rows)
    
    # Test identification
    identifier = DynamicModelIdentifier({'L': L})
    Kv_estimated = identifier.estimate_understeer_gradient(df)
    
    error = abs(Kv_estimated - Kv_true)
    logger.info(f"  True Kv: {Kv_true:.6f} rad/(m/s²)")
    logger.info(f"  Estimated Kv: {Kv_estimated:.6f} rad/(m/s²)")
    logger.info(f"  Error: {error:.6f} ({error/Kv_true*100:.2f}%)")
    
    if 'Kv_r2' in identifier.identified_params:
        logger.info(f"  R²: {identifier.identified_params['Kv_r2']:.4f}")
    
    # Check if error is within acceptable range
    if error < 0.002:  # Less than 8% error
        logger.info("✓ PASS: Ackermann correction working correctly")
        return True
    else:
        logger.error("✗ FAIL: Ackermann correction issue detected")
        return False


def test_step_detection_threshold():
    """Test step detection with new thresholds."""
    logger.info("\n" + "=" * 60)
    logger.info("TEST 2: Step Detection Threshold")
    logger.info("=" * 60)
    
    # Add src to path
    sys.path.append(os.path.join(os.path.dirname(__file__), 'src'))
    from preprocessing import Segmentation
    
    # Create test data with small step
    data_rows = []
    t = 0
    dt = 0.02  # 50 Hz
    
    # Pre-step (1 second)
    for _ in range(50):
        data_rows.append({
            'timestamp': t,
            'steering_angle': 0.0,
            'steering_angle_deg': 0.0,
            'vehicle_speed': 11.11,  # 40 km/h
            'yaw_rate': 0.0,
            'scenario_time': t
        })
        t += dt
    
    # Step of 2.5 degrees (0.0436 rad)
    step_size = np.radians(2.5)
    for _ in range(200):  # 4 seconds
        data_rows.append({
            'timestamp': t,
            'steering_angle': step_size,
            'steering_angle_deg': 2.5,
            'vehicle_speed': 11.11,
            'yaw_rate': 0.1,
            'scenario_time': t
        })
        t += dt
    
    df = pd.DataFrame(data_rows)
    
    # Test segmentation
    segmenter = Segmentation()
    step_segments = segmenter.extract_step_segments(df)
    
    logger.info(f"  Test step size: 2.5° (0.0436 rad)")
    logger.info(f"  Vehicle speed: 40 km/h")
    logger.info(f"  Detected segments: {len(step_segments)}")
    
    if len(step_segments) > 0:
        seg = step_segments[0]
        logger.info(f"  Step magnitude: {np.degrees(seg['step_magnitude']):.2f}°")
        logger.info("✓ PASS: Small step (2.5°) detected at 40 km/h")
        return True
    else:
        logger.error("✗ FAIL: Small step not detected with new threshold")
        return False


def test_import_fixes():
    """Test that imports work without errors."""
    logger.info("\n" + "=" * 60)
    logger.info("TEST 3: Import Error Fixes")
    logger.info("=" * 60)
    
    try:
        # Add src to path
        src_path = os.path.join(os.path.dirname(__file__), 'src')
        if src_path not in sys.path:
            sys.path.insert(0, src_path)
        
        # Import our renamed data_io module
        from data_io import CSVLoader, DataAggregator
        
        # Create instances to verify they work
        loader = CSVLoader()
        aggregator = DataAggregator()
        
        logger.info("✓ PASS: Imports successful, no errors")
        return True
        
    except ImportError as e:
        logger.error(f"✗ FAIL: Import error: {str(e)}")
        return False
    except Exception as e:
        logger.error(f"✗ FAIL: Unexpected error: {str(e)}")
        return False


def test_frequency_range():
    """Test frequency range extension."""
    logger.info("\n" + "=" * 60)
    logger.info("TEST 4: Frequency Range Extension")
    logger.info("=" * 60)
    
    # Check if the segmentation criteria includes extended range
    sys.path.append(os.path.join(os.path.dirname(__file__), 'src'))
    from preprocessing import Segmentation
    
    segmenter = Segmentation()
    freq_range = segmenter.criteria['sweep']['freq_range']
    
    logger.info(f"  Frequency range: {freq_range[0]} - {freq_range[1]} Hz")
    
    if freq_range[1] >= 2.5:
        logger.info("✓ PASS: Frequency range extended to 2.5 Hz")
        return True
    else:
        logger.error(f"✗ FAIL: Frequency range not extended (max: {freq_range[1]} Hz)")
        return False


def main():
    """Run all verification tests."""
    logger.info("\n" + "=" * 60)
    logger.info("LATERAL SYSTEM ID - P0 FIXES VERIFICATION")
    logger.info("=" * 60)
    
    results = []
    
    # Run tests
    results.append(("Ackermann Correction", test_ackermann_correction()))
    results.append(("Step Detection", test_step_detection_threshold()))
    results.append(("Import Fixes", test_import_fixes()))
    results.append(("Frequency Range", test_frequency_range()))
    
    # Summary
    logger.info("\n" + "=" * 60)
    logger.info("SUMMARY")
    logger.info("=" * 60)
    
    passed = sum(1 for _, result in results if result)
    total = len(results)
    
    for test_name, result in results:
        status = "✓ PASS" if result else "✗ FAIL"
        logger.info(f"  {test_name}: {status}")
    
    logger.info(f"\nTotal: {passed}/{total} tests passed")
    
    if passed == total:
        logger.info("\n🎉 All P0 fixes verified successfully!")
        return 0
    else:
        logger.error(f"\n⚠️  {total - passed} tests failed. Please review the fixes.")
        return 1


if __name__ == "__main__":
    sys.exit(main())