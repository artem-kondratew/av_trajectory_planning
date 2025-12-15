import numpy as np

ts, tau = 0.06, 0.2

A = np.array([
    [1.,  ts,         0],
    [0,  1. - ts/tau, 0],
    [0, -1./tau,      0]
])

A_hat = A
print(np.round(A_hat, 3))

for i in range(9):
    A_hat = A_hat @ A
    print(np.round(A_hat, 3))