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

### Python code

[Open test_robot_opt.py](test_opt.py)

