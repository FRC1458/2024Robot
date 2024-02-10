import numpy as np

D = 3 # x, y, r
N = 5 # 5 points
θ = np.random.random((N, 2, D))

μ0 = θ[0]
u = lambda t: np.zeros_like(μ0)
stf = lambda t, s: np.array([[1, t-s], [0, 1]])

# (N, 2, D)
μ1 = np.array([stf(t, 0) @ μ0 for t in range(N)])

# (N, N, 2, 2) * (N, 2, D) -> (N, 2, D)
A = np.zeros((N, N, 2, 2))
i, j, _, _ = np.indices(A.shape)
for t in range(N):
    A[(i == j + t).all(3).all(2)] = [stf(s + t, s) for s in range(N-t)]
μ2 = np.tensordot(A, np.array([μ0] + [u(t) for t in range(1, N)]), axes=[(1, 3), (0, 1)])

print((μ1 == μ2).all())

a = np.zeros((4, 2, 7, 1))
b = np.zeros((0, 4, 7, 4))
print((np.tensordot(a, b, axes=[(0, 2), (1, 2)])).shape) # (2, 1, 0, 4) <Remaining>

a = np.zeros((4, 2, 7, 1, 9))
b = np.zeros((0, 4, 7, 9, 4))
print((np.tensordot(a, b, axes=[(0, 2, 4), (1, 2, 3)])).shape) # (2, 1, 0, 4)