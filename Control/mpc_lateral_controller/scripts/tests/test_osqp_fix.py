#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Test OSQP P matrix fix
"""

import numpy as np
import scipy.sparse as sparse
import osqp

# Parameters
Np = 75
Nc = 10
nx = 2
nu = 1

# Build test P matrix
Q = np.diag([200.0, 250.0])
P_term = 10.0 * Q

# State block
P_X_blocks = []
for k in range(Np):
    P_X_blocks.append(2 * Q)
P_X_blocks.append(2 * P_term)
P_X = sparse.block_diag(P_X_blocks, format='csc')

# Control block
R = 1.0
R_delta = 40.0
T = np.zeros((Nc, Nc))
for i in range(Nc):
    T[i, i] = 1
    if i > 0:
        T[i, i-1] = -1
P_U_dense = 2 * (R * np.eye(Nc) + R_delta * T.T @ T)
P_U = sparse.csc_matrix(P_U_dense)

# Slack block with tiny values
n_slack = 2 * (Np + 1)
P_S = sparse.eye(n_slack, format='csc')

# Full P matrix
P_full = sparse.block_diag([P_X, P_U, P_S], format='csc')

# Set slack values to tiny positive
P_slack_indices = np.arange(P_X.nnz + P_U.nnz, P_X.nnz + P_U.nnz + P_S.nnz)
P_full.data[P_slack_indices] = 1e-10

print(f"Initial P matrix: nnz={P_full.nnz}")

# Create simple test problem
n_vars = 314
n_constraints = 100

q = np.zeros(n_vars)
A = sparse.eye(n_constraints, n_vars, format='csc')
l = -np.ones(n_constraints)
u = np.ones(n_constraints)

# Setup OSQP
solver = osqp.OSQP()
solver.setup(P=P_full, q=q, A=A, l=l, u=u, verbose=False)

print(f"OSQP setup complete")

# Try to update P with different Q values
Q_new = np.diag([5.0, 200.0])  # Inside band Q
P_X_blocks_new = []
for k in range(Np):
    P_X_blocks_new.append(2 * Q_new)
P_X_blocks_new.append(2 * 10.0 * Q_new)
P_X_new = sparse.block_diag(P_X_blocks_new, format='csc')

# Update P data
P_new_data = P_full.data.copy()
P_state_indices = np.arange(P_X.nnz)
P_new_data[P_state_indices] = P_X_new.data

print(f"New P data: length={len(P_new_data)}")

# Try update
try:
    solver.update(Px=P_new_data)
    print("✓ P matrix update successful!")
except Exception as e:
    print(f"✗ P matrix update failed: {e}")

# Try solving
try:
    result = solver.solve()
    if result.info.status == 'solved':
        print("✓ Solver successful after update")
    else:
        print(f"✗ Solver status: {result.info.status}")
except Exception as e:
    print(f"✗ Solve failed: {e}")