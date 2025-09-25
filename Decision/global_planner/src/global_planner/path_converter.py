#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Packaged Path Converter utilities for import as global_planner.path_converter."""

from typing import Tuple
import pandas as pd
import numpy as np
import os


class PathConverter:
    def __init__(self):
        self.stats = {}

    def txt_to_dataframe(self, input_txt_path: str) -> pd.DataFrame:
        if not os.path.exists(input_txt_path):
            raise FileNotFoundError(f"Input file not found: {input_txt_path}")
        data = []
        with open(input_txt_path, 'r', encoding='utf-8') as f:
            for line_num, line in enumerate(f, 1):
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                parts = line.split()
                if len(parts) < 2:
                    continue
                x, y = float(parts[0]), float(parts[1])
                u = float(parts[2]) if len(parts) > 2 else 0.0
                extra = [float(p) for p in parts[3:]] if len(parts) > 3 else []
                row = [line_num, x, y, u] + extra
                data.append(row)
        if not data:
            raise ValueError("No valid data found in input file")
        columns = ['index', 'e', 'n', 'u']
        if len(data[0]) > 4:
            for i in range(4, len(data[0])):
                columns.append(f'extra_{i-3}')
        df = pd.DataFrame(data, columns=columns[: len(data[0])])
        self.stats = {
            'total_points': len(df),
            'file_size': os.path.getsize(input_txt_path),
            'e_range': [df['e'].min(), df['e'].max()],
            'n_range': [df['n'].min(), df['n'].max()],
            'u_range': [df['u'].min(), df['u'].max()] if 'u' in df.columns else None,
            'path_length': self._calculate_path_length(df),
            'avg_spacing': self._calculate_path_length(df) / (len(df) - 1) if len(df) > 1 else 0,
        }
        return df

    def dataframe_to_csv(self, df: pd.DataFrame, output_csv_path: str) -> None:
        os.makedirs(os.path.dirname(output_csv_path), exist_ok=True)
        df.to_csv(output_csv_path, index=False, float_format='%.6f')

    def _calculate_path_length(self, df: pd.DataFrame) -> float:
        if len(df) < 2:
            return 0.0
        dx = np.diff(df['e'].values)
        dy = np.diff(df['n'].values)
        distances = np.sqrt(dx**2 + dy**2)
        return float(np.sum(distances))

    def validate_path_data(self, df: pd.DataFrame) -> Tuple[bool, str]:
        if df.empty:
            return False, "DataFrame is empty"
        required = ['e', 'n']
        if not all(col in df.columns for col in required):
            return False, f"Missing required columns: {required}"
        if df[['e', 'n']].isna().any().any():
            return False, "Path contains NaN values"
        if not np.isfinite(df[['e', 'n']].values).all():
            return False, "Path contains infinite values"
        if len(df) < 2:
            return False, "Path must have at least 2 points"
        return True, "Path data is valid"

    def print_statistics(self) -> None:
        if not self.stats:
            return
        # Intentionally terse for packaged variant
        pass

