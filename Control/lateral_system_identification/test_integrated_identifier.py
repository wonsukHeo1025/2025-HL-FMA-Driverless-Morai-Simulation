#!/usr/bin/env python3
"""
Test script for the integrated lateral system identifier.

This script demonstrates how to use the integrated system identifier
to process multiple CSV files and extract vehicle dynamics parameters.
"""

import os
import sys
import logging

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


def test_integrated_identifier():
    """Test the integrated system identifier with sample data."""
    
    # Import the identifier
    from integrated_system_identifier import IntegratedSystemIdentifier
    
    # Data directory path
    data_dir = "/home/user1/catkin_ws/src/Control/data_collection/src/data_collection/data"
    
    # Check if directory exists
    if not os.path.exists(data_dir):
        logger.error(f"Data directory not found: {data_dir}")
        return False
        
    # Vehicle parameters (Ioniq5)
    vehicle_params = {
        'm': 1800.0,    # kg
        'lf': 1.3,      # m
        'lr': 1.575,    # m
        'L': 2.875      # m
    }
    
    logger.info("=" * 60)
    logger.info("INTEGRATED LATERAL SYSTEM IDENTIFICATION TEST")
    logger.info("=" * 60)
    
    # Create identifier
    logger.info("\n1. Creating system identifier...")
    identifier = IntegratedSystemIdentifier(vehicle_params)
    
    # Process directory
    logger.info(f"\n2. Processing data directory: {data_dir}")
    try:
        results = identifier.process_directory(data_dir)
        
        if results:
            logger.info("\n3. Identification Results:")
            logger.info("-" * 40)
            for param, value in results.items():
                if isinstance(value, float):
                    logger.info(f"  {param}: {value:.4f}")
                else:
                    logger.info(f"  {param}: {value}")
                    
            # Save results
            output_file = "test_identified_parameters.yaml"
            logger.info(f"\n4. Saving results to {output_file}")
            identifier.save_results(output_file)
            
            # Print summary
            logger.info("\n5. Data Summary:")
            logger.info("-" * 40)
            metadata = identifier.aggregator.get_metadata()
            if metadata:
                logger.info(f"  Total samples: {metadata.get('total_samples', 0)}")
                logger.info(f"  Duration: {metadata.get('duration_seconds', 0):.1f} seconds")
                logger.info(f"  Scenarios: {metadata.get('scenarios', [])}")
                
                if 'speed_range' in metadata:
                    speed_min, speed_max = metadata['speed_range']
                    logger.info(f"  Speed range: {speed_min*3.6:.1f} - {speed_max*3.6:.1f} km/h")
                    
                if 'steering_range' in metadata:
                    steer_min, steer_max = metadata['steering_range']
                    logger.info(f"  Steering range: {steer_min:.1f} - {steer_max:.1f} deg")
                    
            logger.info("\n" + "=" * 60)
            logger.info("TEST COMPLETED SUCCESSFULLY")
            logger.info("=" * 60)
            
            return True
        else:
            logger.error("No results obtained from identification")
            return False
            
    except Exception as e:
        logger.error(f"Error during identification: {str(e)}")
        import traceback
        traceback.print_exc()
        return False


def test_individual_components():
    """Test individual components of the system."""
    logger.info("\n" + "=" * 60)
    logger.info("TESTING INDIVIDUAL COMPONENTS")
    logger.info("=" * 60)
    
    # Add src to path
    sys.path.append(os.path.join(os.path.dirname(__file__), 'src'))
    
    # Test CSV loader
    logger.info("\n1. Testing CSV Loader...")
    from io import CSVLoader
    
    loader = CSVLoader()
    test_file = "/home/user1/catkin_ws/src/Control/data_collection/src/data_collection/data/20250811_155411_step_steer.csv"
    
    if os.path.exists(test_file):
        df = loader.load_file(test_file)
        if df is not None:
            logger.info(f"  ✓ Loaded {len(df)} rows from {os.path.basename(test_file)}")
            logger.info(f"  ✓ Columns: {df.columns.tolist()[:5]}... ({len(df.columns)} total)")
        else:
            logger.error("  ✗ Failed to load CSV file")
    else:
        logger.warning(f"  Test file not found: {test_file}")
        
    # Test data aggregator
    logger.info("\n2. Testing Data Aggregator...")
    from io import DataAggregator
    
    aggregator = DataAggregator()
    data_dir = "/home/user1/catkin_ws/src/Control/data_collection/src/data_collection/data"
    
    if os.path.exists(data_dir):
        dataframes = loader.load_directory(data_dir, pattern="*step_steer*.csv")
        if dataframes:
            combined = aggregator.aggregate(dataframes)
            logger.info(f"  ✓ Aggregated {len(dataframes)} files into {len(combined)} samples")
            logger.info(aggregator.summary())
        else:
            logger.warning("  No dataframes to aggregate")
    else:
        logger.warning(f"  Data directory not found: {data_dir}")
        
    # Test signal mapper
    logger.info("\n3. Testing Signal Mapper...")
    from preprocessing import SignalMapper
    
    mapper = SignalMapper()
    if df is not None and len(df) > 0:
        df_mapped = mapper.map_signals(df)
        new_signals = set(df_mapped.columns) - set(df.columns)
        if new_signals:
            logger.info(f"  ✓ Added signals: {list(new_signals)[:5]}")
        
        # Check unit conversion
        if 'steering_angle' in df_mapped.columns and 'steering_angle_deg' in df.columns:
            deg_val = df['steering_angle_deg'].iloc[0]
            rad_val = df_mapped['steering_angle'].iloc[0]
            logger.info(f"  ✓ Unit conversion: {deg_val:.2f}° → {rad_val:.4f} rad")
    else:
        logger.warning("  No data available for signal mapping test")
        
    # Test signal filter
    logger.info("\n4. Testing Signal Filter...")
    from preprocessing import SignalFilter
    
    filter = SignalFilter()
    if df is not None and len(df) > 100:
        df_filtered = filter.process(df_mapped if 'df_mapped' in locals() else df)
        logger.info(f"  ✓ Filtered: {len(df)} → {len(df_filtered)} samples")
        
        reduction = (1 - len(df_filtered)/len(df)) * 100
        logger.info(f"  ✓ Data reduction: {reduction:.1f}%")
    else:
        logger.warning("  Insufficient data for filter test")
        
    # Test segmentation
    logger.info("\n5. Testing Segmentation...")
    from preprocessing import Segmentation
    
    segmenter = Segmentation()
    if 'combined' in locals() and len(combined) > 0:
        # Test steady-state extraction
        steady_segments = segmenter.extract_steady_state_segments(combined)
        logger.info(f"  ✓ Steady-state segments: {len(steady_segments)}")
        
        # Test step extraction
        step_segments = segmenter.extract_step_segments(combined)
        logger.info(f"  ✓ Step response segments: {len(step_segments)}")
        
        # Get statistics
        if steady_segments:
            stats = segmenter.get_segment_statistics(steady_segments)
            logger.info(f"  ✓ Total steady-state duration: {stats.get('total_duration', 0):.1f}s")
    else:
        logger.warning("  No data available for segmentation test")
        
    logger.info("\n" + "=" * 60)
    logger.info("COMPONENT TESTS COMPLETED")
    logger.info("=" * 60)


if __name__ == "__main__":
    # Test individual components first
    test_individual_components()
    
    # Then test integrated system
    success = test_integrated_identifier()
    
    if success:
        logger.info("\n✓ All tests passed successfully!")
        sys.exit(0)
    else:
        logger.error("\n✗ Some tests failed")
        sys.exit(1)