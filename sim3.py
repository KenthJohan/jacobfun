import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Slider

r_A = np.array([-1.0, -1.0])
r_B = np.array([1.0, 0.0])
r_C = np.array([0.5, 1.0])

v_A = np.array([0.01, 0.02])
v_B = np.array([-0.01, 0.01])
v_C = np.array([0.02, -0.01])

def compute_jacobian(r_A, r_B, r_C):
    L1 = np.linalg.norm(r_C - r_A)
    L2 = np.linalg.norm(r_C - r_B)

    if L1 == 0 or L2 == 0:
        raise ValueError("Points are too close, leading to division by zero.")

    dL1_dA = -(r_C - r_A) / L1
    dL1_dC = (r_C - r_A) / L1

    dL2_dB = -(r_C - r_B) / L2
    dL2_dC = (r_C - r_B) / L2

    J = np.zeros((2, 6))
    J[0, 0:2] = dL1_dA
    J[0, 4:6] = dL1_dC
    J[1, 2:4] = dL2_dB
    J[1, 4:6] = dL2_dC

    return J

fig, ax = plt.subplots()
plt.subplots_adjust(left=0.1, bottom=0.25)
ax.set_xlim(-1, 2)
ax.set_ylim(-1, 2)
ax.set_aspect('equal')

line_AC, = ax.plot([], [], 'r-o', label="Actuator AC")
line_BC, = ax.plot([], [], 'b-o', label="Actuator BC")
point_A, = ax.plot([], [], 'go', label="Point A")
point_B, = ax.plot([], [], 'mo', label="Point B")
point_C, = ax.plot([], [], 'ko', label="Point C")

ax_slider = plt.axes([0.1, 0.1, 0.8, 0.03], facecolor='lightgoldenrodyellow')
slider = Slider(ax_slider, 'r_A X', -1.0, 2.0, valinit=r_A[0])

def update(frame):
    global r_A, r_B, r_C

    r_A[0] = slider.val

    #r_A += v_A
    #r_B += v_B
    r_C += v_C

    for point, velocity in zip([r_A, r_B, r_C], [v_A, v_B, v_C]):
        if not (-1 <= point[0] <= 2):
            velocity[0] *= -1
        if not (-1 <= point[1] <= 2):
            velocity[1] *= -1

    line_AC.set_data([r_A[0], r_C[0]], [r_A[1], r_C[1]])
    line_BC.set_data([r_B[0], r_C[0]], [r_B[1], r_C[1]])

    point_A.set_data([r_A[0]], [r_A[1]])
    point_B.set_data([r_B[0]], [r_B[1]])
    point_C.set_data([r_C[0]], [r_C[1]])

    try:
        J = compute_jacobian(r_A, r_B, r_C)
        J_inv = np.linalg.pinv(J)
        print(f"Frame {frame}:\nJacobian Matrix:\n{J_inv}\n")
    except ValueError as e:
        print(f"Frame {frame}: {e}")

    return line_AC, line_BC, point_A, point_B, point_C,

slider.on_changed(lambda val: update(0))

ani = FuncAnimation(fig, update, frames=np.arange(0, 100), interval=100)
ax.legend()
plt.show()
