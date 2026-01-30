import numpy as np
from scipy.optimize import minimize
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter

# ----------------------------
# Parameters
# ----------------------------
T = 5.0
dt = 0.1
N = int(T/dt) + 1

v_max = 5.0
r_robot = 0.5

start = np.array([1.0, 1.0])
goal = np.array([6.0, 6.0])

obstacles_data = [{'pos0':[2.6,4], 'vel':[0.5,0], 'r':0.5},
                {'pos0':[2,2], 'vel':[0.5,0], 'r':0.5}               
                ]

# ----------------------------
# Precompute obstacle positions
# ----------------------------
obstacles_pos = []
for obs in obstacles_data:
    pos_list = []
    pos = np.array(obs['pos0'], dtype=float)
    vel = np.array(obs['vel'], dtype=float)
    for k in range(N):
        pos_list.append(pos.copy())
        pos = pos + vel*dt
    obstacles_pos.append(np.array(pos_list))
obstacles_r = np.array([obs['r'] for obs in obstacles_data])

# ----------------------------
# Initial guess: straight line
# ----------------------------
x0 = np.linspace(start[0], goal[0], N)
y0 = np.linspace(start[1], goal[1], N)
z0 = np.hstack([x0, y0])  # stacked variable

# ----------------------------
# Objective
# ----------------------------
def objective(z):
    x = z[:N]
    y = z[N:]
    return np.sum((x[1:] - x[:-1])**2 + (y[1:] - y[:-1])**2)

# ----------------------------
# Constraints
# ----------------------------
cons = []

# Start & goal
cons.append({'type': 'eq', 'fun': lambda z: z[0] - start[0]})
cons.append({'type': 'eq', 'fun': lambda z: z[N-1] - goal[0]})
cons.append({'type': 'eq', 'fun': lambda z: z[N] - start[1]})
cons.append({'type': 'eq', 'fun': lambda z: z[2*N-1] - goal[1]})

# Velocity constraints
for i in range(N-1):
    def vel_con(z, i=i):
        x = z[:N]
        y = z[N:]
        dx = x[i+1] - x[i]
        dy = y[i+1] - y[i]
        return v_max*dt - np.sqrt(dx**2 + dy**2)
    cons.append({'type': 'ineq', 'fun': vel_con})

# Collision avoidance
for i in range(N):
    for j in range(len(obstacles_data)):
        def col_con(z, i=i, j=j):
            x = z[:N]
            y = z[N:]
            dx = x[i] - obstacles_pos[j][i,0]
            dy = y[i] - obstacles_pos[j][i,1]
            return dx**2 + dy**2 - (0.05+r_robot + obstacles_r[j])**2
        cons.append({'type': 'ineq', 'fun': col_con})

# ----------------------------
# Solve
# ----------------------------
res = minimize(objective, z0, method='SLSQP', constraints=cons, options={'ftol':1e-4, 'disp':True, 'maxiter':500})

if not res.success:
    print("Optimization failed:", res.message)
    exit(1)

z_opt = res.x
x_opt = z_opt[:N]
y_opt = z_opt[N:]
path_opt = np.stack([x_opt, y_opt], axis=1)

# ----------------------------
# Animation
# ----------------------------
fig, ax = plt.subplots()
ax.set_xlim(0,12)
ax.set_ylim(0,8)
ax.set_aspect('equal')

# Goal
ax.plot(goal[0], goal[1], 'gx', markersize=10, label='Goal')

# Robot
robot_patch = plt.Circle(start, r_robot, color='blue')
ax.add_patch(robot_patch)

# Obstacles
obs_patches = []
for j in range(len(obstacles_data)):
    patch = plt.Circle(obstacles_pos[j][0], obstacles_r[j], color='red')
    obs_patches.append(patch)
    ax.add_patch(patch)

# Path
path_line, = ax.plot([], [], 'b--', lw=1)

def update(frame):
    robot_patch.center = path_opt[frame]
    path_line.set_data(path_opt[:frame+1,0], path_opt[:frame+1,1])
    for j, patch in enumerate(obs_patches):
        patch.center = obstacles_pos[j][frame]
    return [robot_patch]+obs_patches+[path_line]

ani = FuncAnimation(fig, update, frames=N, interval=100, blit=False)
ani.save("robot_path_scipy.gif", writer=PillowWriter(fps=10))
print("Animation saved as robot_path_scipy.gif")
plt.legend()
plt.show()

