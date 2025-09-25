#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Debug P matrix size issue
"""

import numpy as np
import scipy.sparse as sparse

# Config values
Np = 75
Nc = 10
n_slack = 2 * (Np + 1)  # 152

# Q matrices
Q_init = np.diag([200.0, 250.0])  # Initial Q from config
Q_inside = np.diag([5.0, 200.0])  # Inside band Q
Q_outside = np.diag([200.0, 250.0])  # Outside band Q

# Build initial P_X (what happens during initialization)
P_X_blocks_init = []
for k in range(Np):
    P_X_blocks_init.append(2 * Q_init)
P_X_blocks_init.append(2 * 10.0 * Q_init)  # Terminal
P_X_init = sparse.block_diag(P_X_blocks_init, format='csc')

# Control block P_U
R = 1.0
R_delta = 40.0
T = np.zeros((Nc, Nc))
for i in range(Nc):
    T[i, i] = 1
    if i > 0:
        T[i, i-1] = -1
P_U_dense = 2 * (R * np.eye(Nc) + R_delta * T.T @ T)
P_U = sparse.csc_matrix(P_U_dense)

# Slack block P_S
P_S = sparse.eye(n_slack, format='csc')

# Full P matrix
P_full_init = sparse.block_diag([P_X_init, P_U, P_S], format='csc')

print(f"Initial P matrix:")
print(f"  P_X: nnz={P_X_init.nnz}, shape={P_X_init.shape}")
print(f"  P_U: nnz={P_U.nnz}, shape={P_U.shape}")
print(f"  P_S: nnz={P_S.nnz}, shape={P_S.shape}")
print(f"  P_full: nnz={P_full_init.nnz}, shape={P_full_init.shape}")
print(f"  Total nnz: {P_X_init.nnz} + {P_U.nnz} + {P_S.nnz} = {P_full_init.nnz}")

# Now build P_X with inside Q (what happens in update_and_solve)
P_X_blocks_inside = []
for k in range(Np):
    P_X_blocks_inside.append(2 * Q_inside)
P_X_blocks_inside.append(2 * 10.0 * Q_inside)  # Terminal
P_X_inside = sparse.block_diag(P_X_blocks_inside, format='csc')

print(f"\nP_X with inside Q:")
print(f"  nnz={P_X_inside.nnz}, shape={P_X_inside.shape}")

# Compare
print(f"\nDifference in nnz: {P_X_inside.nnz} - {P_X_init.nnz} = {P_X_inside.nnz - P_X_init.nnz}")

# Check if the issue is with time-varying scales
q_scales = np.ones(Np)
q_scales[:25] = 0.7  # Front
q_scales[-25:] = 1.3  # Back
p_scale_end = 1.5

# With scales
P_X_blocks_inside_scaled = []
for k in range(Np):
    P_X_blocks_inside_scaled.append(2 * q_scales[k] * Q_inside)
P_X_blocks_inside_scaled.append(2 * p_scale_end * 10.0 * Q_inside)
P_X_inside_scaled = sparse.block_diag(P_X_blocks_inside_scaled, format='csc')

print(f"\nP_X with inside Q and time-varying scales:")
print(f"  nnz={P_X_inside_scaled.nnz}, shape={P_X_inside_scaled.shape}")

# Check data array sizes
print(f"\nData array sizes:")
print(f"  P_X_init.data.size = {P_X_init.data.size}")
print(f"  P_X_inside.data.size = {P_X_inside.data.size}")
print(f"  P_X_inside_scaled.data.size = {P_X_inside_scaled.data.size}")

# The issue might be with how indices are calculated
P_state_nnz = P_X_init.nnz
P_control_nnz = P_U.nnz
P_slack_nnz = P_S.nnz
P_state_indices = np.arange(P_state_nnz)

print(f"\nIndex ranges:")
print(f"  P_state_indices: 0 to {P_state_nnz-1} ({len(P_state_indices)} elements)")
print(f"  Trying to update with {P_X_inside.nnz} elements")
print(f"  ERROR if {P_X_inside.nnz} > {P_state_nnz}")