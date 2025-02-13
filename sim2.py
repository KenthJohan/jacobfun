import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Set up fixed points A and B
A = np.array([-1, -2])
B = np.array([1, 3])

# Calculate vector between A and B
dx = B[0] - A[0]
dy = B[1] - A[1]
D = np.hypot(dx, dy)  # Distance between A and B
theta = np.arctan2(dy, dx)  # Angle of AB with x-axis

# Initialize figure
fig, ax = plt.subplots()
ax.set_xlim(-3, 3)
ax.set_ylim(-3, 6)
ax.set_aspect('equal')
ax.grid(True)
ax.set_title("Modified Linear Actuator Mechanism")

# Initialize lines and point
line_AC, = ax.plot([], [], 'b-', lw=2, label='Actuator AC')
line_BC, = ax.plot([], [], 'r-', lw=2, label='Actuator BC')
point_C, = ax.plot([], [], 'ko', ms=10, label='Point C')
plt.legend()

# Initial actuator lengths (feasible values)
l1_0 = 3.0
l2_0 = 3.0

def compute_position(l1, l2):
    # Calculate position in rotated coordinate system
    u = (l1**2 + D**2 - l2**2) / (2 * D)
    v_sq = l1**2 - u**2
    v = np.sqrt(v_sq) if v_sq >= 0 else 0
    
    # Rotate back to original coordinate system
    x = u * np.cos(theta) - v * np.sin(theta) + A[0]
    y = u * np.sin(theta) + v * np.cos(theta) + A[1]
    return np.array([x, y])

def init():
    line_AC.set_data([], [])
    line_BC.set_data([], [])
    point_C.set_data([], [])
    return line_AC, line_BC, point_C,

def update(frame):
    t = frame * 0.1  # Time parameter
    
    # Update actuator lengths (sinusoidal example)
    l1 = l1_0 + 0.5 * np.sin(t)
    l2 = l2_0 + 0.5 * np.cos(t)
    
    # Compute position of C
    C = compute_position(l1, l2)
    x, y = C
    
    # Compute actual lengths (for Jacobian)
    computed_l1 = np.hypot(x - A[0], y - A[1])
    computed_l2 = np.hypot(x - B[0], y - B[1])
    
    # Jacobian matrix
    J = np.array([
        [(x - A[0])/computed_l1, (y - A[1])/computed_l1],
        [(x - B[0])/computed_l2, (y - B[1])/computed_l2]
    ])
    print(f"Frame {frame}: Jacobian =\n{J}\n")
    
    # Update plot elements
    line_AC.set_data([A[0], C[0]], [A[1], C[1]])
    line_BC.set_data([B[0], C[0]], [B[1], C[1]])
    point_C.set_data([C[0]], [C[1]])
    
    return line_AC, line_BC, point_C,

# Create animation
ani = FuncAnimation(fig, update, frames=200, init_func=init, blit=True, interval=50)

plt.show()