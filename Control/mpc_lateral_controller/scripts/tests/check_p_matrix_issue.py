#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Debug P matrix nnz issue in OSQP backend
"""

import numpy as np
import scipy.sparse as sparse

# Parameters
Np = 75
Nc = 10
nx = 2
nu = 1

# Total variable dimensions
n_states = nx * (Np + 1)  # 152
n_controls = nu * Nc      # 10
n_slack = 2 * (Np + 1)    # 152
n_vars = n_states + n_controls + n_slack  # 314

print(f"Variable dimensions:")
print(f"  n_states: {n_states}")
print(f"  n_controls: {n_controls}")
print(f"  n_slack: {n_slack}")
print(f"  n_vars: {n_vars}")

# Build P matrix as in the original code
Q = np.diag([200.0, 250.0])
P_term = 10.0 * Q
R = 1.0
R_delta = 40.0

# State cost block
q_scales_default = np.ones(Np)
p_scale_default = 1.0

P_X_blocks = []
for k in range(Np):
    P_X_blocks.append(2 * (q_scales_default[k] * Q))
P_X_blocks.append(2 * (p_scale_default * P_term))
P_X = sparse.block_diag(P_X_blocks, format='csc')

# Control cost block
T = np.zeros((Nc, Nc))
for i in range(Nc):
    T[i, i] = 1
    if i > 0:
        T[i, i-1] = -1

P_U_dense = 2 * (R * np.eye(Nc) + R_delta * T.T @ T)
P_U = sparse.csc_matrix(P_U_dense)

# Slack penalty block
P_S = sparse.eye(2 * (Np + 1), format='csc')

print(f"\nIndividual blocks:")
print(f"  P_X: nnz={P_X.nnz}, shape={P_X.shape}")
print(f"  P_U: nnz={P_U.nnz}, shape={P_U.shape}")
print(f"  P_S: nnz={P_S.nnz}, shape={P_S.shape}")

# Full P matrix
P_full = sparse.block_diag([P_X, P_U, P_S], format='csc')

print(f"\nFull P matrix (before zeroing slack):")
print(f"  nnz={P_full.nnz}, shape={P_full.shape}")
print(f"  Total expected: {P_X.nnz + P_U.nnz + P_S.nnz}")

# Now simulate what happens when we set slack to 0
P_state_nnz = P_X.nnz
P_control_nnz = P_U.nnz  
P_slack_nnz = P_S.nnz

P_slack_indices = np.arange(P_state_nnz + P_control_nnz,
                            P_state_nnz + P_control_nnz + P_slack_nnz)

# Check what happens if we set slack values to 0
P_data_copy = P_full.data.copy()
P_data_copy[P_slack_indices] = 0.0

# Count actual non-zeros after setting to 0
actual_nnz = np.count_nonzero(P_data_copy)
print(f"\nAfter setting slack to 0:")
print(f"  Actual non-zeros: {actual_nnz}")
print(f"  P_full.nnz still reports: {P_full.nnz}")

# The issue: OSQP may eliminate zeros during setup
P_full_modified = P_full.copy()
P_full_modified.data[P_slack_indices] = 0.0
P_full_modified.eliminate_zeros()  # This is what OSQP might do

print(f"\nAfter eliminate_zeros():")
print(f"  nnz={P_full_modified.nnz}")

print(f"\nThis explains the error:")
print(f"  OSQP setup with P having nnz={P_full_modified.nnz}")
print(f"  Update tries to use nnz={P_full.nnz}")
print(f"  ERROR: {P_full.nnz} > {P_full_modified.nnz}")