import numpy as np
import matplotlib.pyplot as plt

# Robot arm parameters
L1 = 1.0  # Length of link 1
L2 = 1.0  # Length of link 2

# Forward Kinematics: Calculate end effector position given joint angles
def forward_kinematics(theta1, theta2):
    x = L1 * np.cos(theta1) + L2 * np.cos(theta1 + theta2)
    y = L1 * np.sin(theta1) + L2 * np.sin(theta1 + theta2)
    return np.array([x, y])

# Jacobian matrix: Compute the Jacobian for the 2-DOF robot arm
def jacobian(theta1, theta2):
    J11 = -L1 * np.sin(theta1) - L2 * np.sin(theta1 + theta2)
    J12 = -L2 * np.sin(theta1 + theta2)
    J21 = L1 * np.cos(theta1) + L2 * np.cos(theta1 + theta2)
    J22 = L2 * np.cos(theta1 + theta2)
    return np.array([[J11, J12], [J21, J22]])

# Inverse Kinematics: Use Jacobian to iteratively find joint angles for a target position
def inverse_kinematics_transpose_method(target_pos, initial_angles, tolerance=1e-5, max_iterations=1000, alpha=0.1):
    theta = np.array(initial_angles)
    for i in range(max_iterations):
        # Current end effector position
        current_pos = forward_kinematics(theta[0], theta[1])
        # Error between target and current position
        error = target_pos - current_pos
        # Check if error is within tolerance
        if np.linalg.norm(error) < tolerance:
            break
        # Compute Jacobian
        J = jacobian(theta[0], theta[1])
        # Update joint angles using Jacobian transpose method
        theta += alpha * np.dot(J.T, error)
    return theta


# Inverse Kinematics: Use pseudo-inverse of Jacobian in one step
def inverse_kinematics_pinv(target_pos, initial_angles):
    # Current end effector position
    current_pos = forward_kinematics(initial_angles[0], initial_angles[1])
    # Error between target and current position
    error = target_pos - current_pos
    # Compute Jacobian
    J = jacobian(initial_angles[0], initial_angles[1])
    # Compute pseudo-inverse of Jacobian
    J_pseudo_inv = np.linalg.pinv(J)
    # Update joint angles using pseudo-inverse method
    delta_theta = np.dot(J_pseudo_inv, error)
    return initial_angles + delta_theta

# Plot the robot arm
def plot_robot_arm(theta1, theta2, target_pos, current_pos):
    plt.clf()  # Clear the previous plot
    x0, y0 = 0, 0
    x1 = L1 * np.cos(theta1)
    y1 = L1 * np.sin(theta1)
    x2 = x1 + L2 * np.cos(theta1 + theta2)
    y2 = y1 + L2 * np.sin(theta1 + theta2)
    
    plt.plot([x0, x1], [y0, y1], 'b-', linewidth=2)  # Link 1
    plt.plot([x1, x2], [y1, y2], 'r-', linewidth=2)  # Link 2
    plt.plot(x0, y0, 'ko')  # Base
    plt.plot(x1, y1, 'ko')  # Joint 1
    plt.plot(x2, y2, 'go')  # End effector
    plt.plot(target_pos[0], target_pos[1], 'rx')  # Target position
    plt.plot(current_pos[0], current_pos[1], 'mx')  # Current end effector position
    plt.xlim(-2, 2)
    plt.ylim(-2, 2)
    plt.gca().set_aspect('equal')
    plt.draw()

# Mouse movement event handler
def on_mouse_move(event):
    global target_pos
    if event.inaxes:  # Check if the mouse is inside the plot area
        target_pos = np.array([event.xdata, event.ydata])

# Main program
if __name__ == "__main__":
    # Initial joint angles (in radians)
    initial_angles = [np.pi/4, np.pi/4]
    
    # Initial target position
    target_pos = forward_kinematics(initial_angles[0], initial_angles[1])
    current_pos = np.copy(target_pos)  # Current end effector position
    
    # Set up the plot
    fig, ax = plt.subplots()
    plt.connect('motion_notify_event', on_mouse_move)
    
    # Animation loop
    while True:
        # Interpolate current position toward target position
        step_size = 0.05  # Controls the speed of movement
        current_pos += step_size * (target_pos - current_pos)
        
        # Perform inverse kinematics to update joint angles
        # final_angles = inverse_kinematics_transpose_method(current_pos, initial_angles, tolerance=0.1, max_iterations=1, alpha=0.4)
        final_angles = inverse_kinematics_pinv(current_pos, initial_angles)
        
        # Update the plot
        plot_robot_arm(final_angles[0], final_angles[1], target_pos, current_pos)
        
        # Pause to create animation effect
        plt.pause(0.01)
        
        # Update initial angles for the next iteration
        initial_angles = final_angles