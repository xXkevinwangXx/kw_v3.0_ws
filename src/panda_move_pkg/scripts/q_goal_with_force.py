#!/usr/bin/env python3
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ THIS CODE IS OLD ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"
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
# Desciption: This script gets a target end-effector pose from user input, reads force and effort, 
# and minimze joint displacement when solving IK for the target pose. 
# ------------------ Effort and Wrench Callback ------------------
def print_efforts_and_wrenches(record_duration, realtime_print=True):
    efforts = []   # list of tuples: (timestamp, [efforts])
    wrenches = []  # list of tuples: (timestamp, Fx, Fy, Fz, Tx, Ty, Tz)

    def effort_cb(msg):
        ts = time.time()
        eff = list(msg.effort)[:7]   # only first 7 joints
        efforts.append((ts, eff))
        if realtime_print:
            eff_str = ", ".join([f"{e:.3f}" for e in eff])
            ros.loginfo(f"[Effort @ {ts:.3f}] efforts: [{eff_str}]")

    def wrench_cb(msg):
        ts = time.time()
        wrench = getattr(msg, "O_F_ext_hat_K", None)
        if wrench is None or len(wrench) < 6:
            return
        forces = list(wrench[0:3])
        torques = list(wrench[3:6])
        wrenches.append((ts, forces + torques))
        if realtime_print:
            f_str = ", ".join([f"{f:.3f}" for f in forces])
            t_str = ", ".join([f"{t:.3f}" for t in torques])
            f_mag = np.linalg.norm(forces)
            t_mag = np.linalg.norm(torques)
            ros.loginfo(f"[Wrench @ {ts:.3f}] Forces: [{f_mag}]  Torques: [{t_mag}]")

    # create subscribers
    effort_sub = ros.Subscriber('/franka_state_controller/joint_states', JointState, effort_cb) # JointState is from sensors_msgs.msg
    wrench_sub = ros.Subscriber('/franka_state_controller/franka_states', FrankaState, wrench_cb) # FrankaState is from franka_msgs.msg

    ros.loginfo(f"Recording (printing) efforts and wrenches for {record_duration:.1f}s ...")
    start = time.time()
    try:
        while time.time() - start < record_duration and not ros.is_shutdown():
            ros.sleep(0.05)
    finally:
        # unregister subscribers (cleanup)
        try:
            effort_sub.unregister()
        except Exception:
            pass
        try:
            wrench_sub.unregister()
        except Exception:
            pass

    # Summary print
    ros.loginfo("=== Summary: joint efforts ===")
    if efforts:
        arr = np.array([e for (_, e) in efforts])   # shape (N, 7)
        mean = np.mean(arr, axis=0)
        mn = np.min(arr, axis=0)
        mx = np.max(arr, axis=0)
        for i in range(arr.shape[1]):
            ros.loginfo(f"joint{i+1}: mean={mean[i]:.3f}, min={mn[i]:.3f}, max={mx[i]:.3f}")
    else:
        ros.loginfo("No joint effort samples recorded.")

    ros.loginfo("=== Summary: end-effector wrench ===")
    if wrenches:
        arr_w = np.array([w for (_, w) in wrenches])  # shape (N, 6)
        mean_w = np.mean(arr_w, axis=0)
        mn_w = np.min(arr_w, axis=0)
        mx_w = np.max(arr_w, axis=0)
        ros.loginfo(f"Forces mean/min/max: {mean_w[0]:.3f}/{mn_w[0]:.3f}/{mx_w[0]:.3f}, "
                    f"{mean_w[1]:.3f}/{mn_w[1]:.3f}/{mx_w[1]:.3f}, "
                    f"{mean_w[2]:.3f}/{mn_w[2]:.3f}/{mx_w[2]:.3f}")
        ros.loginfo(f"Torques mean/min/max: {mean_w[3]:.3f}/{mn_w[3]:.3f}/{mx_w[3]:.3f}, "
                    f"{mean_w[4]:.3f}/{mn_w[4]:.3f}/{mx_w[4]:.3f}, "
                    f"{mean_w[5]:.3f}/{mn_w[5]:.3f}/{mx_w[5]:.3f}")
    else:
        ros.loginfo("No wrench samples recorded.")

# ------------------ Step 1: Get T_start from real robot state ------------------
#region
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
#endregion

# ------------------ Step 2: Get T_goal from user input ------------------
#region
print("Enter 6 values for target pose: x y z roll pitch yaw (in radians)")
user_input = input(">>> ").strip().split()
if len(user_input) != 6:
    raise ValueError("Please enter exactly 6 values.")

x, y, z, roll, pitch, yaw = map(float, user_input)
T_goal = SE3(x, y, z) * SE3.RPY([roll, pitch, yaw], order='xyz')
#endregion

# ------------------ Step 3: Find Goal Joint Angles while Minimizing Joint Displacement ------------------
#region
q_start = np.array(msg.q[:7])  # Use real robot joint angles directly
panda = rtb.models.Panda()
panda.base = SE3(1.0978, 0.4738, 0.4940)  # Align sim base to real robot base

#region
candidates = []
try:
    sol = panda.ikine_LM(T_goal, q0=q_start)
except TypeError:
    sol = panda.ikine_LM(T_goal)
if sol.success:
    candidates.append(sol.q)

# Try multiple random start positions to find better IK solutions
n_restarts = 10
noise_scale = 0.15
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
#endregion

# sol_goal = panda.ikine_LM(T_goal)
# if not sol_goal.success:
#     raise RuntimeError("IK failed for the goal pose.")
# q_goal = sol_goal.q
#endregion

# ------------------ Step 4: Generate trajectory ------------------
#region
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
#endregion

# ------------------ Step 6: Export trajectory to CSV ------------------
#region
np.savetxt("joint_angles.csv", q_all, delimiter=",")
print("Saved joint angles to joint_angles.csv")

# ------------------ Step 7: Ask user for hardware execution ------------------
confirm = input("Do you want to move the real robot with this trajectory? [y/n]: ").strip().lower()
if confirm == 'y':
    # print efforts & wrenches for 3 seconds BEFORE executing the trajectory
    try:
        print_efforts_and_wrenches(record_duration=1.0, realtime_print=True)
    except Exception as e:
        ros.logwarn(f"Failed to record/print efforts/wrenches: {e}")

    # now run the trajectory follower (actual execution)
    os.system("rosrun panda_move_pkg follow_traj.py --traj_file joint_angles.csv")
elif confirm == 'n':
    print("Exiting without moving robot.")
else:
    print("Invalid input. Exiting.")
#endregion