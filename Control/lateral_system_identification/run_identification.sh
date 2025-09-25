#!/bin/bash
#
# Run lateral system identification on CSV data
#
# Usage: ./run_identification.sh [data_directory] [options]
#

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Default values
DEFAULT_DATA_DIR="/home/user1/catkin_ws/src/Control/data_collection/src/data_collection/data"
OUTPUT_FILE="identified_parameters.yaml"
PYTHON_CMD="python3"

# Function to print colored output
print_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Function to check dependencies
check_dependencies() {
    print_info "Checking dependencies..."
    
    # Check Python
    if ! command -v $PYTHON_CMD &> /dev/null; then
        print_error "Python3 not found!"
        exit 1
    fi
    
    # Check required Python packages
    $PYTHON_CMD -c "import numpy" 2>/dev/null || {
        print_warning "numpy not installed. Installing..."
        pip3 install numpy
    }
    
    $PYTHON_CMD -c "import pandas" 2>/dev/null || {
        print_warning "pandas not installed. Installing..."
        pip3 install pandas
    }
    
    $PYTHON_CMD -c "import scipy" 2>/dev/null || {
        print_warning "scipy not installed. Installing..."
        pip3 install scipy
    }
    
    $PYTHON_CMD -c "import yaml" 2>/dev/null || {
        print_warning "pyyaml not installed. Installing..."
        pip3 install pyyaml
    }
    
    print_info "Dependencies OK"
}

# Function to display help
show_help() {
    echo "Lateral System Identification Tool"
    echo ""
    echo "Usage: $0 [data_directory] [options]"
    echo ""
    echo "Options:"
    echo "  -h, --help          Show this help message"
    echo "  -o, --output FILE   Output YAML file (default: identified_parameters.yaml)"
    echo "  -m, --mass VALUE    Vehicle mass in kg (default: 1901)"
    echo "  --lf VALUE          Front axle to CG distance in meters (default: 1.7)"
    echo "  --lr VALUE          Rear axle to CG distance in meters (default: 1.3)"
    echo "  --plot              Generate plots"
    echo "  --save-plots DIR    Save plots to directory"
    echo "  -v, --verbose       Verbose output"
    echo "  --test              Run test script instead"
    echo ""
    echo "Examples:"
    echo "  $0                                    # Use default data directory"
    echo "  $0 /path/to/data                      # Specify data directory"
    echo "  $0 /path/to/data --plot -o params.yaml # With plots and custom output"
    echo "  $0 --test                              # Run test script"
}

# Main script
main() {
    # Check if help is requested
    if [[ "$1" == "-h" ]] || [[ "$1" == "--help" ]]; then
        show_help
        exit 0
    fi
    
    # Check if test mode
    if [[ "$1" == "--test" ]]; then
        print_info "Running test script..."
        cd "$(dirname "$0")"
        $PYTHON_CMD test_integrated_identifier.py
        exit $?
    fi
    
    # Check dependencies
    check_dependencies
    
    # Parse arguments
    DATA_DIR="${1:-$DEFAULT_DATA_DIR}"
    shift
    
    # Check if data directory exists
    if [ ! -d "$DATA_DIR" ]; then
        print_error "Data directory not found: $DATA_DIR"
        echo "Please specify a valid directory containing CSV files."
        exit 1
    fi
    
    # Count CSV files
    CSV_COUNT=$(find "$DATA_DIR" -maxdepth 1 -name "*.csv" 2>/dev/null | wc -l)
    if [ "$CSV_COUNT" -eq 0 ]; then
        print_error "No CSV files found in $DATA_DIR"
        exit 1
    fi
    
    print_info "Found $CSV_COUNT CSV files in $DATA_DIR"
    
    # Change to script directory
    cd "$(dirname "$0")"
    
    # Build command
    CMD="$PYTHON_CMD integrated_system_identifier.py \"$DATA_DIR\" $@"
    
    # Run identification
    print_info "Starting lateral system identification..."
    echo -e "${YELLOW}Command:${NC} $CMD"
    echo ""
    
    # Execute
    eval $CMD
    EXIT_CODE=$?
    
    # Check result
    if [ $EXIT_CODE -eq 0 ]; then
        print_info "Identification completed successfully!"
        
        # Check if output file was created
        if [ -f "$OUTPUT_FILE" ]; then
            print_info "Results saved to: $OUTPUT_FILE"
            echo ""
            echo "Quick summary:"
            echo "---------------"
            # Show first few lines of results
            head -n 20 "$OUTPUT_FILE" | grep -E "^\s*(Kv|Iz|Caf|Car):" || echo "No parameters found"
        fi
    else
        print_error "Identification failed with exit code $EXIT_CODE"
        exit $EXIT_CODE
    fi
}

# Run main function
main "$@"