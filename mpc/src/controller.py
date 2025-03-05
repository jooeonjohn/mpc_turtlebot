import numpy as np

def mpc_controller(idx, A, B, C, w, X0, Np, U_ref, Q, R, U_min, U_max):
    """
    MPC Controller for mobile robots.
    """
    
    # Find the M, N, W matrices
    M, N, W = mpc_gain(idx, A, B, C, w, Np)
    n = B.shape[1]  # number of inputs
    
    # Constraints for control input
    
    Lb = []
    Ub = []
    for j in range(Np):  # boundary for control input
        Lb.extend(U_min - U_ref[j, :])
        Ub.extend(U_max - U_ref[j, :])
    Lb = np.vstack(Lb)  # Combine into a single 2D array
    Ub = np.vstack(Ub)
    
    b = np.vstack([Ub, -Lb])  # Concatenate Ub and -Lb vertically
    C1 = np.eye(n * Np)
    A_cons = np.vstack([C1, -C1])
    # For constrained problem
    H = 2 * (N.T @ Q @ N + R)
    f = 2 * (N.T @ Q @ M @ X0) + 2 * (N.T @ Q @ W)
    # Find control input using Hildreth's quadratic programming
    deltaU = hildreth_qp(H, f, A_cons, b)

    U = deltaU[:n] + U_ref[0,:].reshape(-1, 1)
    vel = U[0].item()
    omega = U[1].item()
    
    return vel, omega


def mpc_gain(idx, A, B, C, w, Np):
    """
    Compute M, N, and W matrices for the MPC problem.
    """
    m1, _ = C.shape
    n_in = B.shape[1]
    
    M = []
    A_e = C @ A
    
    # Matrix M
    for k in range(Np):
        A_e = A_e @ A
        M.append(A_e)
    M = np.vstack(M)

    # Matrix N
    v = C @ B
    for k in range(1, Np):
        v = np.vstack([v, A_e @ B])

    N = np.zeros((m1 * Np, Np * n_in))
    N[:, :n_in] = v
    for j in range(2, Np+1):
        N[:, (j - 1) * n_in:j * n_in] = np.vstack([np.zeros((m1 * (j - 1), n_in)), v[:(Np - j + 1) * m1, :]])
    
    # Matrix W
    L = np.zeros((m1 * Np, Np * m1))
    L[:, :m1] = M
    for j in range(2, Np+1):
        L[:, (j - 1) * m1:(j) * m1] = np.vstack([np.zeros((m1 * (j - 1), m1)), M[:(Np - j + 1) * m1, :]])
    
    Res = np.vstack([w for j in range(Np)])

    W = L @ Res
    
    return M, N, W


def hildreth_qp(H, f, A_cons, b):
    """
    Hildreth's method for quadratic programming.
    """
    n1, m1 = A_cons.shape
    
    # Solution of the unconstrained problem
    x = -np.linalg.inv(H) @ f
    kk = 0
    # Check if all constraints are satisfied
    for i in range(n1):
        if A_cons[i, :] @ x > b[i]:
            kk += 1
    
    if kk == 0:  # If no constraints are violated, solution found
        return x
    
    # Define matrices for Hildreth's optimization method
    P = A_cons @ np.linalg.inv(H) @ A_cons.T
    d = A_cons @ np.linalg.inv(H) @ f + b
    
    # Initial guess for lambda
    lambda_ = np.zeros(d.shape)
    al = 10  # stop tolerance
    
    for m in range(38):
        lambda_p = lambda_.copy()
        for i in range(n1):
            w = P[i, :] @ lambda_ - P[i, i] * lambda_[i]
            w = w + d[i]
            lambda_[i] = max(0, -w / P[i, i])  # if lambda_i is negative, set it to 0
        
        al = np.sum((lambda_ - lambda_p) ** 2)
        if al < 1e-8:
            break
    
    x = -np.linalg.inv(H) @ f - np.linalg.inv(H) @ A_cons.T @ lambda_
    return x
