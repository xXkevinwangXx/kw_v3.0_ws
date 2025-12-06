#!/usr/bin/env python3
import sys
import rospy as ros
import os
import time
import argparse
import numpy as np
import pickle

from actionlib import SimpleActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, FollowJointTrajectoryResult

import roboticstoolbox as rtb
from spatialmath import SE3
import swift
from franka_msgs.msg import FrankaState

# ------------------ Step 1: Get T_start from real robot state ------------------
ros.init_node('simulate_and_execute_traj')
msg = ros.wait_for_message('/franka_state_controller/franka_states', FrankaState)
#T real robot frame
print("T Real Robot Frame")
if len(msg.O_T_EE) != 16:
    raise ValueError(f"O_T_EE has {len(msg.O_T_EE)} elements; expected 16.")
T_start = np.array(msg.O_T_EE).reshape((4, 4), order='F')
print(T_start)
T_start = SE3(T_start[0,3], T_start[1,3], T_start[2,3]) * SE3.OA(T_start[:3, 1], T_start[:3, 2])
print("Simulatead Robot values:")
print(T_start)
xyz = T_start.t
rpy = T_start.rpy(order='xyz')
print(f"T_start position: x={xyz[0]:.4f}, y={xyz[1]:.4f}, z={xyz[2]:.4f}")
print(f"T_start orientation (RPY): roll={rpy[0]:.4f}, pitch={rpy[1]:.4f}, yaw={rpy[2]:.4f}")


# ------------------ Step 2: Get T_goal from user input ------------------
print("Enter 6 values for target pose: x y z roll pitch yaw (in radians)")
user_input = input(">>> ").strip().split()
if len(user_input) != 6:
    raise ValueError("Please enter exactly 6 values.")

x, y, z, roll, pitch, yaw = map(float, user_input)
T_goal = SE3(x, y, z) * SE3.RPY([roll, pitch, yaw], order='xyz')

# ------------------ Step 3: Inverse Kinematics ------------------
q_start = np.array(msg.q[:7])  # Use real robot joint angles directly
panda = rtb.models.Panda()
panda.base = SE3(1.0978, 0.4738, 0.4940)  # Align sim base to real robot base

# vvvvv NEW CODE FOR Minimizing Joint Movement vvvvv
candidates = []
try:
    sol = panda.ikine_LM(T_goal, q0=q_start)
except TypeError:
    sol = panda.ikine_LM(T_goal)
if sol.success:
    candidates.append(sol.q)

n_restarts = 20
noise_scale = 0.5
rng = np.random.default_rng(0)

for i in range(n_restarts):
    q0 = q_start + noise_scale * rng.uniform(-1.0, 1.0, size=7)
    try:
        s = panda.ikine_LM(T_goal, q0=q0)
    except TypeError:
        s = panda.ikine_LM(T_goal)
    if s.success:
        candidates.append(s.q)

if len(candidates) == 0:
    raise RuntimeError("IK failed for the goal pose with all attempts.")

candidates = np.array(candidates)
dists = np.linalg.norm(candidates - q_start, axis=1)
best_idx = np.argmin(dists)
q_goal = candidates[best_idx]
print(f"Selected IK solution (index {best_idx}) with ||Δq|| = {dists[best_idx]:.4f} rad")
# ^^^^^ NEW CODE FOR Minimizing Joint Movement ^^^^^


# sol_goal = panda.ikine_LM(T_goal)
# if not sol_goal.success:
#     raise RuntimeError("IK failed for the goal pose.")
# q_goal = sol_goal.q

# # ------------------ Step 4: Generate trajectory ------------------
traj = rtb.jtraj(q_start, q_goal, 200)
q_all = traj.q[:, :7]  # Only first 7 joints (exclude gripper if present)

# ------------------ Step 5: Simulate in Swift ------------------
env = swift.Swift()
env.launch(realtime=True)
env.add(panda)

for q in traj.q:
    panda.q = q
    env.step(0.02)

time.sleep(1)

# ------------------ Step 6: Export trajectory to CSV ------------------
np.savetxt("joint_angles.csv", q_all, delimiter=",")
print("Saved joint angles to joint_angles.csv")

# ------------------ Step 7: Ask user for hardware execution ------------------
confirm = input("Do you want to move the real robot with this trajectory? [y/n]: ").strip().lower()
if confirm == 'y':
    os.system("rosrun panda_move_pkg follow_traj.py --traj_file joint_angles.csv")
elif confirm == 'n':
    print("Exiting without moving robot.")
else:
    print("Invalid input. Exiting.")
