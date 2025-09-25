#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Helper module to suppress OSQP C-level warnings.
"""

import os
import sys
import contextlib


@contextlib.contextmanager
def suppress_stdout_stderr():
    """Context manager to suppress stdout and stderr at C level."""
    # Save original file descriptors
    old_stdout = os.dup(1)
    old_stderr = os.dup(2)
    
    try:
        # Open devnull
        devnull = os.open(os.devnull, os.O_WRONLY)
        
        # Redirect stdout and stderr to devnull
        os.dup2(devnull, 1)
        os.dup2(devnull, 2)
        
        # Close devnull fd
        os.close(devnull)
        
        yield
        
    finally:
        # Restore original stdout and stderr
        os.dup2(old_stdout, 1)
        os.dup2(old_stderr, 2)
        
        # Close saved descriptors
        os.close(old_stdout)
        os.close(old_stderr)


@contextlib.contextmanager
def suppress_stderr_only():
    """Context manager to suppress only stderr at C level."""
    # Save original stderr
    old_stderr = os.dup(2)
    
    try:
        # Open devnull
        devnull = os.open(os.devnull, os.O_WRONLY)
        
        # Redirect stderr to devnull
        os.dup2(devnull, 2)
        
        # Close devnull fd
        os.close(devnull)
        
        yield
        
    finally:
        # Restore original stderr
        os.dup2(old_stderr, 2)
        
        # Close saved descriptor
        os.close(old_stderr)