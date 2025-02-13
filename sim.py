import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# Crane parameters
L1 = 5.0        # Length of Boom 1
d1 = 1.0        # Offset from B to C on Boom 1
D1 = np.array([2.0, 0.0])  # Base connection for Cylinder 1

L2 = 4.0        # Length of Boom 2
d2 = 1.0        # Offset from BB's end to C2 on Boom 2
D2_rel = 3.0    # D2 is located 3 units from A along Boom 1

# Initialize angles
theta = np.radians(30)  # Angle of Boom 1
phi = np.radians(-45)   # Angle of Boom 2 relative to Boom 1

# Setup figure
fig, ax = plt.subplots()
ax.set_xlim(-5, 10)
ax.set_ylim(-5, 10)
ax.set_aspect('equal')
ax.grid(True)

# Initialize lines
boom1_line, = ax.plot([], [], 'b-', lw=4)
boom2_line, = ax.plot([], [], 'r-', lw=4)
cylinder1_line, = ax.plot([], [], 'g-', lw=2)
cylinder2_line, = ax.plot([], [], 'm-', lw=2)

def init():
    boom1_line.set_data([], [])
    boom2_line.set_data([], [])
    cylinder1_line.set_data([], [])
    cylinder2_line.set_data([], [])
    return boom1_line, boom2_line, cylinder1_line, cylinder2_line

def update(frame):
    # Animate angles
    theta = np.radians(30 + 20 * np.sin(frame * 0.1))
    phi = np.radians(-45 + 30 * np.sin(frame * 0.05))

    # Boom 1 (A to B)
    A = np.array([0.0, 0.0])
    B = A + np.array([L1 * np.cos(theta), L1 * np.sin(theta)])
    boom1_line.set_data([A[0], B[0]], [A[1], B[1]])

    # Cylinder 1 (D1 to C1)
    C1 = B - np.array([d1 * np.cos(theta), d1 * np.sin(theta)])
    cylinder1_line.set_data([D1[0], C1[0]], [D1[1], C1[1]])

    # Boom 2 (BB to B2)
    BB = B  # Joint BB is at the end of Boom 1
    D2 = A + (B - A) * (D2_rel / L1)  # D2 moves with Boom 1
    B2 = BB + np.array([L2 * np.cos(theta + phi), L2 * np.sin(theta + phi)])
    boom2_line.set_data([BB[0], B2[0]], [BB[1], B2[1]])

    # Cylinder 2 (D2 to C2)
    C2 = B2 - np.array([d2 * np.cos(theta + phi), d2 * np.sin(theta + phi)])
    cylinder2_line.set_data([D2[0], C2[0]], [D2[1], C2[1]])

    return boom1_line, boom2_line, cylinder1_line, cylinder2_line

# Animate
ani = animation.FuncAnimation(fig, update, frames=np.arange(0, 314), 
                              init_func=init, blit=True, interval=50)

plt.show()