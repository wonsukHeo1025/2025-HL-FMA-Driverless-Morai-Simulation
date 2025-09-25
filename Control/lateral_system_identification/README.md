# Lateral System Identification Tool

## Overview

This tool provides comprehensive lateral dynamics parameter identification for autonomous vehicles using data collected from MORAI simulator. It processes multiple CSV files containing vehicle test data (steady-state cornering, step steer, sine sweep) to identify key parameters for MPC controller design.

## Features

- **Multi-scenario Processing**: Handles steady-state, step response, and frequency response data
- **Automatic Data Aggregation**: Processes multiple CSV files from a directory
- **Signal Processing Pipeline**: Filtering, resampling, and outlier removal
- **Parameter Identification**:
  - Understeer gradient (Kv)
  - Yaw moment of inertia (Iz)
  - Cornering stiffness (Caf, Car)
- **Quality Metrics**: Validation and confidence scoring
- **Visualization**: Plots for verification and analysis

## Quick Start

### 1. Basic Usage

```bash
# Make script executable
chmod +x run_identification.sh

# Run with default settings (uses default data directory)
./run_identification.sh

# Specify custom data directory
./run_identification.sh /path/to/csv/data/

# Run with plots
./run_identification.sh /path/to/data/ --plot
```

### 2. Python Direct Usage

```bash
# Run the integrated identifier
python3 integrated_system_identifier.py /path/to/data/ --output params.yaml --plot

# Run test script
python3 test_integrated_identifier.py
```

## Installation

### Prerequisites

- Python 3.8+
- ROS Noetic (optional, for integration with ROS system)

### Required Python Packages

```bash
pip3 install numpy pandas scipy matplotlib pyyaml
```

Or install from requirements file:
```bash
pip3 install -r requirements.txt
```

## Data Requirements

### CSV File Structure

The tool expects CSV files with the following columns:

**Required columns:**
- `timestamp` - Unix timestamp
- `steering_angle_deg` - Steering angle in degrees
- `true_velocity_x` - Longitudinal velocity in m/s
- `imu_accel_y` - Lateral acceleration in m/s²
- `yaw_rate` - Yaw rate in rad/s
- `scenario_type` - Type of test scenario
- `scenario_time` - Time since scenario start
- `is_steady_state` - Boolean flag for steady-state

**Optional columns:**
- `x_position`, `y_position` - Vehicle position
- `gps_latitude`, `gps_longitude` - GPS coordinates
- `estimated_velocity_y` - Lateral velocity

### File Naming Convention

Files should be named with scenario type for automatic detection:
- `*_steady_state_cornering.csv`
- `*_step_steer.csv`
- `*_sine_sweep.csv`

## Detailed Usage

### Command Line Options

```bash
python3 integrated_system_identifier.py [options] data_directory

Required arguments:
  data_directory        Directory containing CSV files

Optional arguments:
  -h, --help           Show help message
  -o, --output FILE    Output YAML file (default: identified_parameters.yaml)
  
Vehicle parameters:
  -m, --mass VALUE     Vehicle mass in kg (default: 1800)
  --lf VALUE           Front axle to CG distance in m (default: 1.3)
  --lr VALUE           Rear axle to CG distance in m (default: 1.575)
  
Processing options:
  --scenario TYPE      Process specific scenario: all|steady_state|step_steer|sine_sweep
  --min-speed VALUE    Minimum speed threshold in m/s (default: 0.5)
  --max-steering VALUE Maximum steering angle in deg (default: 20)
  
Output options:
  --plot              Generate plots
  --save-plots DIR    Save plots to directory
  -v, --verbose       Verbose output
```

### Example Commands

```bash
# Process all scenarios with custom vehicle parameters
python3 integrated_system_identifier.py ./data/ \
    --mass 1850 --lf 1.35 --lr 1.525 \
    --output ioniq5_params.yaml --plot

# Process only steady-state data
python3 integrated_system_identifier.py ./data/ \
    --scenario steady_state \
    --output steady_state_params.yaml

# Verbose mode with plots saved
python3 integrated_system_identifier.py ./data/ \
    --verbose --plot --save-plots ./results/plots/
```

## Output Format

### YAML Output Structure

```yaml
timestamp: '2025-08-11T15:30:00'
vehicle_parameters:
  m: 1800.0      # [kg] Vehicle mass
  lf: 1.3        # [m] Front axle to CG
  lr: 1.575      # [m] Rear axle to CG
  L: 2.875       # [m] Wheelbase

identified_parameters:
  # From steady-state analysis
  Kv: 0.0234     # [rad/(m/s²)] Understeer gradient
  
  # From step response
  Iz: 2456.7     # [kg·m²] Yaw inertia
  Caf: 98500.0   # [N/rad] Front cornering stiffness
  Car: 115000.0  # [N/rad] Rear cornering stiffness
  
  # From frequency response
  Iz_freq: 2510.3
  Caf_freq: 101000.0
  Car_freq: 118000.0

quality_metrics:
  steady_state_r2: 0.982
  step_rmse: 0.018
  freq_points: 15

metadata:
  total_samples: 150000
  duration_seconds: 3600.5
  scenarios: ['steady_state_cornering', 'step_steer', 'sine_sweep']
```

## Architecture

### Module Structure

```
lateral_system_identification/
├── src/
│   ├── io/                  # Data loading and aggregation
│   │   ├── csv_loader.py    # CSV file loading with validation
│   │   └── data_aggregator.py # Combine multiple CSV files
│   ├── preprocessing/        # Signal processing
│   │   ├── signal_mapping.py # Unit conversion and mapping
│   │   ├── filtering.py     # Filtering and resampling
│   │   └── segmentation.py  # Extract test segments
│   ├── identification/       # Parameter identification algorithms
│   │   ├── steady_state_id.py
│   │   ├── step_response_id.py
│   │   └── frequency_id.py
│   ├── validation/          # Model validation
│   └── reporting/           # Output generation
├── integrated_system_identifier.py  # Main entry point
├── test_integrated_identifier.py    # Test script
└── run_identification.sh            # Bash wrapper
```

### Processing Pipeline

1. **Data Loading**: Load and validate CSV files
2. **Aggregation**: Combine multiple files, sort by timestamp
3. **Signal Mapping**: Convert units (deg→rad), add derived signals
4. **Filtering**: Remove outliers, apply lowpass filter, resample
5. **Segmentation**: Extract scenario-specific segments
6. **Identification**: Apply algorithms for each scenario type
7. **Validation**: Cross-validate results
8. **Output**: Generate YAML and plots

## Algorithms

### Steady-State Identification

Estimates understeer gradient using:
```
δ = L/R + Kv*ay
```
Where:
- δ = steering angle
- L = wheelbase
- R = turning radius
- Kv = understeer gradient
- ay = lateral acceleration

### Step Response Identification

Uses optimization to fit dynamic bicycle model:
```
State-space: ẋ = Ax + Bu
Output: y = Cx
```
Identifies Iz, Caf, Car by minimizing yaw rate prediction error.

### Frequency Response Identification

Matches measured frequency response to model:
```
H(ω) = Yaw_rate(ω) / Steering(ω)
```
Uses Welch's method for FRF estimation.

## Troubleshooting

### Common Issues

1. **No CSV files found**
   - Check data directory path
   - Verify CSV files exist and have correct extension

2. **Missing required columns**
   - Ensure CSV files have all required columns
   - Check column names match exactly

3. **Poor identification results**
   - Verify data quality (sufficient speed, steering variation)
   - Check for sensor noise or dropouts
   - Adjust filter parameters

4. **Memory issues with large files**
   - Process files in batches
   - Increase system RAM
   - Use downsampling

## Integration with ROS

To integrate with ROS MPC system:

```python
#!/usr/bin/env python3
import rospy
import yaml
from std_msgs.msg import String

# Load identified parameters
with open('identified_parameters.yaml', 'r') as f:
    params = yaml.safe_load(f)

# Use in MPC controller
Iz = params['identified_parameters']['Iz']
Caf = params['identified_parameters']['Caf']
Car = params['identified_parameters']['Car']
```

## Contributing

Contributions are welcome! Please ensure:
- Code follows Python PEP8 style
- Add unit tests for new features
- Update documentation

## License

This project is part of the autonomous vehicle control system for MORAI simulator.

## Contact

For questions or issues, please contact the Autonomous Vehicle Control Team.