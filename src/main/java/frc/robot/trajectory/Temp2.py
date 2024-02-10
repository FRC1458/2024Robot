import numpy as np

D = 3 # x, y, r
N = 5 # 5 points

# (N, 2, D) * (N, N, 2, 2) * (N, 2, D)
# (D, N, 2) * (N, 2, D)
# ()

θ = np.zeros((2, D))
μ = np.zeros((N, 2, D))
KInv = np.zeros((N, N, 2, 2))

d = θ - μ
print(d.shape, KInv.shape, d.shape)

a = np.tensordot(d, KInv, axes=[(0, 1), (1, 3)])
print(a.shape, d.shape)

b = np.tensordot(a, d, axes=[(0, 1, 2), (2, 0, 1)])
print(b, b.shape)

# Mahalanobis distance transpose in tensordot axes selection
a = np.random.random((3, 5, 9))
b = np.random.random((3, 2, 5))
x1 = np.tensordot(a, b, axes=[(0, 1), (0, 2)])
x2 = np.tensordot(a, b.transpose(0, 2, 1), axes=[(0, 1), (0, 1)])
print((x1 == x2).all())