#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Check sparsity patterns of Q matrices
"""

import numpy as np
import scipy.sparse as sparse

# Test Q matrices from config
q_inside = [5.0, 200.0]
q_outside = [200.0, 250.0]

Q_inside = np.diag(q_inside)
Q_outside = np.diag(q_outside)

print("Q_inside:")
print(Q_inside)
print(f"Shape: {Q_inside.shape}, Non-zeros: {np.count_nonzero(Q_inside)}")

print("\nQ_outside:")
print(Q_outside)
print(f"Shape: {Q_outside.shape}, Non-zeros: {np.count_nonzero(Q_outside)}")

# Test sparse representation
Np = 75  # From config

# Build P_X blocks with inside Q
P_X_blocks_inside = []
for k in range(Np):
    P_X_blocks_inside.append(2 * Q_inside)
P_X_blocks_inside.append(2 * 10.0 * Q_inside)  # Terminal
P_X_inside = sparse.block_diag(P_X_blocks_inside, format='csc')

# Build P_X blocks with outside Q
P_X_blocks_outside = []
for k in range(Np):
    P_X_blocks_outside.append(2 * Q_outside)
P_X_blocks_outside.append(2 * 10.0 * Q_outside)  # Terminal
P_X_outside = sparse.block_diag(P_X_blocks_outside, format='csc')

print(f"\nP_X_inside: nnz={P_X_inside.nnz}, shape={P_X_inside.shape}")
print(f"P_X_outside: nnz={P_X_outside.nnz}, shape={P_X_outside.shape}")

# Check if sparsity patterns are the same
same_pattern = (P_X_inside.nnz == P_X_outside.nnz)
print(f"\nSame sparsity pattern: {same_pattern}")

# Now test with time-varying scales
q_scales = np.ones(Np)
q_scales[:int(Np/3)] = 0.7  # Front
q_scales[-int(Np/3):] = 1.3  # Back

# Build with scales
P_X_blocks_inside_scaled = []
for k in range(Np):
    P_X_blocks_inside_scaled.append(2 * q_scales[k] * Q_inside)
P_X_blocks_inside_scaled.append(2 * 1.5 * 10.0 * Q_inside)  # Terminal with p_scale
P_X_inside_scaled = sparse.block_diag(P_X_blocks_inside_scaled, format='csc')

P_X_blocks_outside_scaled = []
for k in range(Np):
    P_X_blocks_outside_scaled.append(2 * q_scales[k] * Q_outside)
P_X_blocks_outside_scaled.append(2 * 1.5 * 10.0 * Q_outside)  # Terminal with p_scale
P_X_outside_scaled = sparse.block_diag(P_X_blocks_outside_scaled, format='csc')

print(f"\nWith time-varying scales:")
print(f"P_X_inside_scaled: nnz={P_X_inside_scaled.nnz}, shape={P_X_inside_scaled.shape}")
print(f"P_X_outside_scaled: nnz={P_X_outside_scaled.nnz}, shape={P_X_outside_scaled.shape}")
print(f"Same scaled sparsity pattern: {P_X_inside_scaled.nnz == P_X_outside_scaled.nnz}")

# Check the actual values to see if any become "zero" due to numerical precision
min_inside = np.min(np.abs(P_X_blocks_inside_scaled[0]))
min_outside = np.min(np.abs(P_X_blocks_outside_scaled[0]))
print(f"\nMin absolute values in first block:")
print(f"Inside: {min_inside}")
print(f"Outside: {min_outside}")