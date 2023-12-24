import numpy as np

def dh(alpha, a, d, theta):
    return np.array([
        [np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
        [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
        [0, np.sin(alpha), np.cos(alpha), d],
        [0, 0, 0, 1]
    ])

def fwd_kinematics(theta1, theta2, theta3, theta4, theta5):
    # Define DH parameters
    alpha = [0, 0, 0, 0, 0]
    a = [0.08, 0.01, -0.145, 0.115, 0]
    d = [0.07, 0.25, 0, 0, -0.15]
    
    # Calculate transformation matrices
    T01 = dh(alpha[0], a[0], d[0], theta1)
    T12 = dh(alpha[1], a[1], d[1], theta2)
    T23 = dh(alpha[2], a[2], d[2], theta3)
    T34 = dh(alpha[3], a[3], d[3], theta4)
    T45 = dh(alpha[4], a[4], d[4], theta5)

    # Calculate the overall transformation matrix
    T05 = np.dot(np.dot(np.dot(T01, T12), T23), np.dot(T34, T45))

    return T05[:3, 3]

def jacobian(theta1, theta2, theta3, theta4, theta5):
    # Small perturbation for numerical differentiation
    delta = 1e-6

    # Calculate the end-effector position without joint perturbation
    p0 = fwd_kinematics(theta1, theta2, theta3, theta4, theta5)

    # Numerical differentiation to compute columns of the Jacobian
    dp1 = (fwd_kinematics(theta1 + delta, theta2, theta3, theta4, theta5) - p0) / delta
    dp2 = (fwd_kinematics(theta1, theta2 + delta, theta3, theta4, theta5) - p0) / delta
    dp3 = (fwd_kinematics(theta1, theta2, theta3 + delta, theta4, theta5) - p0) / delta
    dp4 = (fwd_kinematics(theta1, theta2, theta3, theta4 + delta, theta5) - p0) / delta
    dp5 = (fwd_kinematics(theta1, theta2, theta3, theta4, theta5 + delta) - p0) / delta

    # Assemble the Jacobian matrix
    J = np.column_stack((dp1, dp2, dp3, dp4, dp5))

    return J

def inverse_kinematics(xd, theta_init, max_iterations=100, tolerance=1e-6, alpha=0.1):
    theta = theta_init.copy()

    for i in range(max_iterations):
        # Calculate the current end-effector position
        x_current = fwd_kinematics(*theta)

        # Compute the error
        error = xd - x_current

        # Check convergence
        if np.linalg.norm(error) < tolerance:
            print(f"Converged in {i+1} iterations.")
            break

        # Update joint angles using the Jacobian transpose method
        J = jacobian(*theta)
        delta_theta = np.dot(J.T, error)
        theta += alpha * delta_theta

    return theta

if __name__ == "__main__":
    # Desired end-effector position
    xd = np.array([0.5, 0.5, 0.5])

    # Initial guess for joint angles
    theta_init = np.array([0.1, 0.2, 0.3, 0.4, 0.5])

    # Solve inverse kinematics
    theta_solution = inverse_kinematics(xd, theta_init)

    # Print the result
    print("Desired end-effector position:", xd)
    print("Calculated joint angles:", theta_solution)
    print("Final end-effector position:", fwd_kinematics(*theta_solution))
