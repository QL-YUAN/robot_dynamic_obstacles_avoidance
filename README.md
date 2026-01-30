# Robot Trajectory Optimization in a Dynamic Obstacle Environment

[Watch the video](dynamic%20obstacle.mp4)

## 1. Introduction

The goal of this work is to compute a **collision-free trajectory** for a circular mobile robot navigating a 2D workspace with **moving obstacles**. The robot must move from a fixed start position to a fixed goal position in a given time horizon, subject to **velocity limits** and **collision avoidance constraints**. The trajectory should be **smooth** and **minimize total path length**, representing an energy-efficient or time-efficient path.

This problem is representative of **autonomous navigation in dynamic environments**, such as mobile robots in warehouses, self-driving cars, or drones in cluttered spaces.

---

## 2. Problem Setup

### 2.1 Robot Model

- The robot is modeled as a **circle** of radius `r_robot`.  
- The robot moves in a 2D plane `(x, y)`.  
- Maximum speed is constrained to `v_max`.

---

### 2.2 Time Discretization

- Total time horizon: `T = 5.0 s`  
- Time step: `dt = 0.1 s`  
- Number of steps: `N = T / dt + 1 = 51`  

The trajectory is represented as sequences:

```

x = [x0, x1, ..., x_{N-1}],
y = [y0, y1, ..., y_{N-1}]

```

---

### 2.3 Obstacles

The environment contains **moving circular obstacles**, each defined by:

- Initial position: `p0`  
- Constant velocity: `v`  
- Radius: `r_obs`

Obstacle positions at time step `i` are computed as:

```

p_i = p0 + i * dt * v

```

In this setup:

| Obstacle | Start Pos | Velocity | Radius |
|----------|-----------|----------|--------|
| 1        | [2.6,4]  | [0.5,0] | 0.5    |
| 2        | [2,2]    | [0.5,0] | 0.5    |

---

## 3. Mathematical Formulation

### 3.1 Decision Variables

```

z = [x0, ..., x_{N-1}, y0, ..., y_{N-1}] ∈ R^{2N}

```

---

### 3.2 Objective Function

Minimize total squared displacement (path length):

```

J(z) = sum_{i=0}^{N-2} ( (x_{i+1} - x_i)^2 + (y_{i+1} - y_i)^2 )

```

---

### 3.3 Constraints

#### 3.3.1 Boundary Conditions

```

x0 = x_start,  y0 = y_start
x_{N-1} = x_goal,  y_{N-1} = y_goal

```

---

#### 3.3.2 Velocity Limits

```

sqrt( (x_{i+1}-x_i)^2 + (y_{i+1}-y_i)^2 ) <= v_max * dt,  for i=0,...,N-2

```

---

#### 3.3.3 Collision Avoidance

The robot must maintain a safe distance from moving obstacles:

```

(x_i - p_{i,j}^x)^2 + (y_i - p_{i,j}^y)^2 >= (r_robot + r_obs_j + epsilon)^2,  ∀ i,j

```

Where `epsilon` is a small safety margin (0.05 m).  

---

## 4. Optimization Method

Solve as a **nonlinear constrained optimization problem**:

```

minimize J(z)
subject to boundary, velocity, and collision constraints

```

### 4.1 Solver

- **Method:** Sequential Least Squares Quadratic Programming (SLSQP)  
- **Initial Guess:** Straight line from start to goal  
- **Constraints:** Implemented as equality and inequality functions in `scipy.optimize.minimize`  
- **Tolerance:** 1e-4, max iterations = 500  

---

## 5. Implementation Notes

- Obstacles’ positions are **precomputed** for each time step.  
- Trajectory is represented as **stacked vectors**: `z = [x0,...,x_{N-1}, y0,...,y_{N-1}]`.  
- Velocity and collision constraints are implemented as **lambda functions** evaluated by SLSQP.  
- Objective function is quadratic in differences, ensuring smooth trajectories.  

---

## 6. Results

- Optimized trajectory **avoids moving obstacles** while satisfying all constraints.  
- Trajectory and obstacles are animated using **Matplotlib’s FuncAnimation**.  
- Robot path and obstacle positions are visualized over time.  
- Output GIF file: `robot_path_scipy.gif`.

---

## 7. Discussion

- Problem is **highly nonlinear** due to collision constraints with moving obstacles.  
- SLSQP may **fail for large or closely spaced obstacles**, highlighting the need for a good initial guess or hybrid/global methods.  
- Suitable for **offline trajectory planning**; for real-time, **MPC** or **sampling-based planners** may be preferred.

---

## 8. Conclusion

- Complete 2D trajectory optimization framework implemented in Python and SciPy.  
- Accounts for **velocity limits, collision avoidance, and boundary conditions**.  
- Resulting trajectory is **smooth, feasible, and collision-free**.  

---

### Optional Improvements
0. The purpose is for **high dimensional** mobile manipulation system
1. Use **global optimization** (differential evolution) for tight environments.  
2. Apply **hybrid DE + SLSQP** to speed convergence and improve feasibility.  
3. Include **Jacobian/gradient computation** for faster refinement.  
```
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

```

```

