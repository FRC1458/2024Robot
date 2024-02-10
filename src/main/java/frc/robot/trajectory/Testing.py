import numpy as np

# K^-T = Transpose of Inverse of K
# Q^-1_i = Inverse of i-th elem of Q

dt = 0.02 # Change in time
D = 3 # x, y, r
N = 5 # 5 points
Qc = np.array(0.5)[None, None]
θ = np.random.random((N, 2, D))

μ0 = θ[0]
K0 = np.zeros((2, 2))
stf = lambda t, s: np.array([[1, t-s], [0, 1]])
μ = np.array([stf(t, 0) @ μ0 for t in range(N)])

F = np.array([[0], [1]])
K = np.zeros((N, N, 2, 2))

def QT(t, tp=0):
    timesteps = int((tp - t) / dt) + 1
    x = np.zeros((timesteps, 2, 2))
    for i in range(timesteps):
        temp = stf(tp, t + i * dt) @ F
        x[i] = temp @ Qc @ temp.T
    return np.trapz(x, dx=dt, axis=0)

# !!! VERY BAD SOLUTION !!!
for t in range(N):
    for tp in range(N):
        K[tp, t] = stf(t, 0) @ K0 @ stf(tp, 0).T + QT(0, min(t, tp))

Q = np.zeros((N, N, 2, 2))
Q[np.eye(N)==1] = [K0] + [QT(t-1, t) for t in range(1, N)]

A = np.zeros((N, N, 2, 2))
for k in range(N):
    A[np.diag([True] * (N-k), -k)] = [stf(t + k, t) for t in range(N-k)]

KInv = A * Q * A.transpose(0, 1, 3, 2)
def P(θ):
    d = θ - μ
    a = np.tensordot(d, KInv, axes=[(0, 1), (1, 3)])
    b = np.tensordot(a, d, axes=[(0, 1, 2), (2, 0, 1)])
    return np.exp(-0.5 * b)

# a = np.random.random((9, 9))
# b = np.random.random((9, 9))
# x1 = a @ b
# x2 = np.tensordot(a, b, axes=[(1,), (0,)])
# print((x1 == x2).all())