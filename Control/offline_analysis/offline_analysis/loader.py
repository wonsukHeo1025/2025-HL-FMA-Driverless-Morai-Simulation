"""Utility for loading CSV logs into a pandas DataFrame."""

from pathlib import Path
from typing import List
import pandas as pd

__all__ = ['load_csv_files']


def _gather_files(paths: List[str]) -> List[Path]:
    files: List[Path] = []
    for p in paths:
        pth = Path(p)
        if pth.is_dir():
            files.extend(sorted(pth.glob('*.csv')))
        elif pth.is_file() and pth.suffix == '.csv':
            files.append(pth)
    if not files:
        raise FileNotFoundError('No CSV files found in given paths')
    return files


def load_csv_files(paths: List[str]) -> pd.DataFrame:
    """Load one or more CSV files into a single DataFrame.

    Parameters
    ----------
    paths : List[str]
        File or directory paths.
    Returns
    -------
    pd.DataFrame
        Concatenated dataframe containing all rows. Adds column `source_file`.
    """
    files = _gather_files(paths)
    frames = []
    for f in files:
        df = pd.read_csv(f)
        df['source_file'] = f.name
        frames.append(df)
    return pd.concat(frames, ignore_index=True)