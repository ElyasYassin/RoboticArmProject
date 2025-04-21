import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize
from mpl_toolkits.mplot3d import Axes3D

# Problem setup
dt = 0.1
N = 20  # time steps
theta_init = np.array([0.0, 0.0, 0.0])  # initial joint angles
theta_target = np.array([1.0, 0.5, -0.5])  # target joint angles

# Dynamics model (simplified): theta_next = theta + torque * dt
def forward_dynamics(theta, torque):
    return theta + torque * dt

# Forward kinematics (mock): just return theta sum for simplicity
def forward_kinematics(theta):
    return np.sum(theta)

# --- SINGLE SHOOTING ---
def simulate_trajectory_single_shooting(torques):
    theta = np.copy(theta_init)
    trajectory = [theta]
    for t in range(N):
        u = torques[t*3:(t+1)*3]
        theta = forward_dynamics(theta, u)
        trajectory.append(theta)
    return np.array(trajectory)

def cost_single_shooting(torques):
    trajectory = simulate_trajectory_single_shooting(torques)
    final_theta = trajectory[-1]
    return np.linalg.norm(final_theta - theta_target)

torques_guess = np.zeros(N * 3)
result_ss = minimize(cost_single_shooting, torques_guess, method='L-BFGS-B')
trajectory_ss = simulate_trajectory_single_shooting(result_ss.x)

# --- DIRECT COLLOCATION ---
def cost_direct_collocation(x):
    total_cost = 0
    for t in range(N):
        theta = x[t*6:t*6+3]
        total_cost += np.linalg.norm(theta - theta_target)**2
    return total_cost

def dynamics_constraint(x):
    constraints = []
    for t in range(N):
        theta_now = x[t*6:t*6+3]
        torque_now = x[t*6+3:t*6+6]
        if t < N-1:
            theta_next = x[(t+1)*6:(t+1)*6+3]
            theta_pred = forward_dynamics(theta_now, torque_now)
            constraints.append(theta_next - theta_pred)
    return np.concatenate(constraints)

x_guess = np.zeros(N * 6)  # [theta1, tau1, theta2, tau2, ...]
constraints = {'type': 'eq', 'fun': dynamics_constraint}
result_dc = minimize(cost_direct_collocation, x_guess, constraints=constraints, method='SLSQP')

# Extract direct collocation trajectory
trajectory_dc = np.array([result_dc.x[i*6:i*6+3] for i in range(N)])

# Plot the results
plt.figure(figsize=(10, 5))
plt.plot(trajectory_ss[:, 0], label='Single Shooting θ1')
plt.plot(trajectory_ss[:, 1], label='Single Shooting θ2')
plt.plot(trajectory_ss[:, 2], label='Single Shooting θ3')
plt.plot(trajectory_dc[:, 0], '--', label='Direct Collocation θ1')
plt.plot(trajectory_dc[:, 1], '--', label='Direct Collocation θ2')
plt.plot(trajectory_dc[:, 2], '--', label='Direct Collocation θ3')
plt.title("Joint Trajectories: Single Shooting vs Direct Collocation")
plt.xlabel("Time Step")
plt.ylabel("Joint Angles")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()

# Forward kinematics for a 3-link 3-DoF planar arm (simple model)
def fk_3d(theta, link_lengths=[1.0, 1.0, 1.0]):
    x, y, z = 0.0, 0.0, 0.0
    T = np.eye(4)
    trajectory = [[x, y, z]]
    for i in range(3):
        l = link_lengths[i]
        rot_z = np.array([
            [np.cos(theta[i]), -np.sin(theta[i]), 0, 0],
            [np.sin(theta[i]),  np.cos(theta[i]), 0, 0],
            [0,                0,                 1, 0],
            [0,                0,                 0, 1],
        ])
        trans_x = np.array([
            [1, 0, 0, l],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
        ])
        T = T @ rot_z @ trans_x
        point = T @ np.array([0, 0, 0, 1])
        trajectory.append(point[:3])
    return np.array(trajectory)

# Get 3D trajectories for each timestep
arm_paths_ss = [fk_3d(theta) for theta in trajectory_ss]
arm_paths_dc = [fk_3d(theta) for theta in trajectory_dc]

# Plotting the 3D arm over time
fig = plt.figure(figsize=(12, 6))
ax = fig.add_subplot(111, projection='3d')
ax.set_title("3D Arm Trajectories: Single Shooting vs Direct Collocation")

# Plot sampled timesteps
for i in range(0, N, 3):
    path_ss = arm_paths_ss[i]
    path_dc = arm_paths_dc[i]
    
    ax.plot(path_ss[:, 0], path_ss[:, 1], path_ss[:, 2], 'b-', alpha=0.5)
    ax.plot(path_dc[:, 0], path_dc[:, 1], path_dc[:, 2], 'r--', alpha=0.5)

# Final poses
ax.plot(arm_paths_ss[-1][:, 0], arm_paths_ss[-1][:, 1], arm_paths_ss[-1][:, 2], 'bo-', label='Single Shooting Final')
ax.plot(arm_paths_dc[-1][:, 0], arm_paths_dc[-1][:, 1], arm_paths_dc[-1][:, 2], 'ro--', label='Direct Collocation Final')

ax.set_xlim(-3, 3)
ax.set_ylim(-3, 3)
ax.set_zlim(-1, 1)
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.legend()
plt.tight_layout()
plt.show()