#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""CSV data logger for data collection."""

import os
import csv
import rospy
from datetime import datetime
from typing import Dict, Any, List, Optional
from collections import OrderedDict


class CSVLogger:
    """General-purpose CSV data logger."""
    
    def __init__(self, log_dir: str = None, scenario_name: str = "data",
                 buffer_size: int = 100):
        """Initialize CSV logger.
        
        Args:
            log_dir: Directory to save logs (default: package data directory)
            scenario_name: Name prefix for log files
            buffer_size: Number of rows to buffer before writing
        """
        # Set up log directory
        if log_dir is None:
            package_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            log_dir = os.path.join(package_path, 'data')
        
        self.log_dir = log_dir
        os.makedirs(self.log_dir, exist_ok=True)
        
        # Create filename with timestamp first for better sorting
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.filename = os.path.join(self.log_dir, 
                                     f"{timestamp}_{scenario_name}.csv")
        
        # Data buffer
        self.buffer = []
        self.buffer_size = buffer_size
        self.headers = None
        self.file_handle = None
        self.csv_writer = None
        
        rospy.loginfo(f"CSV logger initialized. Will save to: {self.filename}")
    
    def log(self, data: Dict[str, Any]):
        """Log a data row.
        
        Args:
            data: Dictionary of data to log
        """
        # Initialize headers on first log
        if self.headers is None:
            self._initialize_headers(data)
        
        # Add timestamp if not present
        if 'timestamp' not in data:
            data['timestamp'] = rospy.Time.now().to_sec()
        
        # Ensure all headers are present
        row = OrderedDict()
        for header in self.headers:
            row[header] = data.get(header, '')
        
        # Add to buffer
        self.buffer.append(row)
        
        # Flush if buffer is full
        if len(self.buffer) >= self.buffer_size:
            self.flush()
    
    def _initialize_headers(self, data: Dict[str, Any]):
        """Initialize CSV headers from first data row.
        
        Args:
            data: Sample data dictionary
        """
        # Ensure timestamp is first
        headers = ['timestamp'] if 'timestamp' not in data else []
        headers.extend([k for k in data.keys() if k != 'timestamp'])
        
        self.headers = headers
        
        # Open file and write headers
        self.file_handle = open(self.filename, 'w', newline='')
        self.csv_writer = csv.DictWriter(self.file_handle, fieldnames=self.headers)
        self.csv_writer.writeheader()
    
    def flush(self):
        """Write buffered data to file."""
        if not self.buffer or not self.csv_writer:
            return
        
        # Write all buffered rows
        self.csv_writer.writerows(self.buffer)
        self.file_handle.flush()
        
        # Clear buffer
        self.buffer = []
    
    def save(self):
        """Save all data and close file."""
        # Flush remaining data
        self.flush()
        
        # Close file
        if self.file_handle:
            self.file_handle.close()
            self.file_handle = None
            self.csv_writer = None
        
        # Log statistics
        rospy.loginfo(f"Data collection complete. Saved to: {self.filename}")
        
        # Get file size
        if os.path.exists(self.filename):
            size = os.path.getsize(self.filename)
            rospy.loginfo(f"File size: {size / 1024:.1f} KB")
            
            # Count rows
            with open(self.filename, 'r') as f:
                row_count = sum(1 for _ in f) - 1  # Subtract header
            rospy.loginfo(f"Total data points saved: {row_count}")
    
    def set_headers(self, headers: List[str]):
        """Manually set CSV headers.
        
        Args:
            headers: List of header names
        """
        if self.headers is not None:
            rospy.logwarn("Headers already initialized, ignoring new headers")
            return
        
        self.headers = headers
        
        # Open file and write headers
        self.file_handle = open(self.filename, 'w', newline='')
        self.csv_writer = csv.DictWriter(self.file_handle, fieldnames=self.headers)
        self.csv_writer.writeheader()
    
    def get_filename(self) -> str:
        """Get the log filename.
        
        Returns:
            Path to log file
        """
        return self.filename
    
    def __del__(self):
        """Ensure file is closed on deletion."""
        if self.file_handle:
            self.save()