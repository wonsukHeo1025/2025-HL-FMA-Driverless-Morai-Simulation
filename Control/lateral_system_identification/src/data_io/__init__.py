"""IO module for data loading and aggregation."""

from .csv_loader import CSVLoader
from .data_aggregator import DataAggregator

__all__ = ['CSVLoader', 'DataAggregator']