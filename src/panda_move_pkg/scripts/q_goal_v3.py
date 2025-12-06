#!/usr/bin/env python3
"""
MMC runner using real robot start pose and user-specified goal pose.

Adapted from the Robotics Toolbox example mmc.py (Peter Corke / Jesse Haviland).
"""
# for saving and replay
import os
# action client imports (for replay option A)
import actionlib
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from sensor_msgs.msg import JointState

import matplotlib.pyplot as plt

import sys
import time
import numpy as np
import swift
import roboticstoolbox as rtb
import spatialmath as sm
from spatialmath import SE3
import qpsolvers as qp

# ROS imports
import rospy as ros
from franka_msgs.msg import FrankaState

# --- helper: build SE3 from FrankaState.O_T_EE field (column-major F order used earlier) ---
def SE3_from_O_T_EE_flat(flat):
    arr = np.array(flat)
    if arr.size != 16:
        raise ValueError(f"O_T_EE has {arr.size} elements; expected 16.")
    M = arr.reshape((4, 4), order='F')  # same ordering as your previous code
    # Build SE3 from translation + orientation columns (OA helper)
    return SE3(M[0, 3], M[1, 3], M[2, 3]) * SE3.OA(M[:3, 1], M[:3, 2])

# --- Init ROS and read real robot state for start pose & joint angles ---
try:
    # avoid exception if already initialized in process
    ros.init_node("mmc_from_real_start", anonymous=True)
except Exception:
    # node may already be running in this process; ignore
    pass

print("Waiting for Franka state message to get current robot pose and joints...")
msg = ros.wait_for_message('/franka_state_controller/franka_states', FrankaState)
print("Received Franka state.")

# build T_start from O_T_EE (world -> EE)
T_start = SE3_from_O_T_EE_flat(msg.O_T_EE)

print("T_start (from robot):")
print(T_start)
print("Start EE pos (xyz):", np.round(T_start.t, 4), "  RPY (rad):", np.round(T_start.rpy(order='xyz'), 4))




# --- Ask the user for the desired goal pose (x y z roll pitch yaw in radians) ---
q_start = np.array(msg.q[:7])  # first 7 entries are robot joints
print("Launching Swift...")
env = swift.Swift()
env.launch()
panda = rtb.models.Panda()

# --- Align simulated panda.base to the real robot pose T_start ---
# Compute the base transform so that panda.fkine(q_start) == T_start
# Set panda.base BEFORE assigning panda.q to keep frames consistent.
panda.base = SE3()                 # temporarily identity for model-local FK
A_model = panda.fkine(q_start)     # model-base -> EE at q_start (model-local)
panda.base = T_start * A_model.inv()

# Now set the simulated robot joints to the real robot joints
panda.q = q_start.copy()

print("Enter 6 values for target pose: x y z roll pitch yaw (radians)")
vals = input(">>> ").strip().split()
if len(vals) != 6:
    print("Error: please enter exactly 6 values (x y z roll pitch yaw). Exiting.")
    sys.exit(1)
x, y, z, roll, pitch, yaw = map(float, vals)

# Build user goal in world coords (what user types)
Tep_world = SE3(x, y, z) * SE3.RPY([roll, pitch, yaw], order='xyz')
print("Desired target transform (Tep_world):")
print(Tep_world)

# Convert user goal into the robot BASE frame (used inside MMC loop)
# Tep = panda.base.inv() * Tep_world
Tep = Tep_world
print("Converted Tep (base frame) used by MMC:")
print(Tep)

time.sleep(1)  # let user see prints if desired

env.add(panda)
n = 7


# Sanity check: simulated fkine at q_start should equal T_start (world frame)
T_sim_world = panda.fkine(q_start)
err = np.linalg.norm(T_sim_world.A - T_start.A)
print(f"Sanity: ||panda.fkine(q_start) - T_start||_F = {err:.6e}")
if err > 1e-6:
    print("Warning: alignment residual > 1e-6 -- check construction of T_start or A_model.")


# ---- MMC controller loop (same form as example but uses T_start/q_start/Tep) ----
arrived = False
iteration = 0

print("Starting MMC loop: press Ctrl-C to abort (use e-stop on real robot when testing on hardware).")
# BEFORE the loop (add)
q_log = []          # list to store joint rows
# for diagnostics / plotting
time_log = []     # seconds since start of MMC
manip_log = []    # manipulability scalar (product of singular values)
start_time = time.time()
dt = 0.05           # keep dt outside the loop; reuse below

try:
    while not arrived:
        iteration += 1
        # world-frame FK (what fkine returns)
        Te_world = panda.fkine(panda.q)   # 4x4 SE3 (world -> EE)

        # convert to robot-base frame (base -> EE)
        Te_base = panda.base.inv() * Te_world

        # use Te_base from here on (instead of Te_world)
        Te = Te_base
        print("Current EE pose (Te):")
        print(Te)
        # Transform from the end-effector to desired pose
        eTep = Te.inv() * Tep
      
        # Spatial error (magnitude used in mmc sample)
        err = np.sum(np.abs(np.r_[eTep.t, eTep.rpy() * np.pi / 180.0]))

        # End-effector spatial velocity to approach goal
        v, arrived = rtb.p_servo(Te, Tep, 1.0)

        # Gain term for control minimization
        Y = 0.01

        # Quadratic component of objective (size n+6)
        Q = np.eye(n + 6)
        Q[:n, :n] *= Y
        Q[n:, n:] = (1.0 / max(err, 1e-6)) * np.eye(6)

        # Equality constraints [J, I] [qd; s] = v
        J = np.asarray(panda.jacobe(panda.q))  # 6 x n
        Aeq = np.c_[J, np.eye(6)]
        beq = v.reshape((6,))

        # compute manipulability (product of singular values) and log it
        s = np.linalg.svd(J, compute_uv=False)        # singular values of J (6 x 7)
        manip = np.prod(s) if np.all(s > 0) else 0.0  # protect against zeros
        t_now = time.time() - start_time
        manip_log.append(manip)
        time_log.append(t_now)


        # Inequality constraints (joint limit damper)
        Ain = np.zeros((n + 6, n + 6))
        bin = np.zeros(n + 6)
        ps = 0.05  # min approach angle to limit
        pi = 0.9   # influence angle
        Ain[:n, :n], bin[:n] = panda.joint_velocity_damper(ps, pi, n)

        # Linear objective term (manipulability Jacobian)
        c = np.r_[-panda.jacobm().reshape((n,)), np.zeros(6)]

        # Bounds on qdot and slack
        lb = -np.r_[panda.qdlim[:n], 10 * np.ones(6)]
        ub = np.r_[panda.qdlim[:n], 10 * np.ones(6)]

        # Solve QP (try 'highs' backend if available)
        try:
            qd_solution = qp.solve_qp(P=Q, q=c, G=Ain, h=bin, A=Aeq, b=beq, lb=lb, ub=ub, solver='highs')
        except TypeError:
            # fallback to positional call signature
            qd_solution = qp.solve_qp(Q, c, Ain, bin, Aeq, beq, lb=lb, ub=ub, solver='highs')

        if qd_solution is None:
            print(f"[mmc] Iter {iteration}: QP returned None, aborting.")
            break

        # Extract joint velocity command
        qd = qd_solution[:n]

        # INSIDE loop replace integrate/step with:
        panda.q = panda.q + dt * qd
        env.step(dt)
        time.sleep(dt)

        # record a copy of the current 7-joint vector
        q_log.append(panda.q[:7].copy())

        # Logging
        if iteration % 5 == 0:
            manip_val = np.prod(np.linalg.svd(J, compute_uv=False))
            print(f"[mmc] it={iteration:03d} | e={err:.6f} | ||qd||={np.linalg.norm(qd):.4f} | manip={manip_val:.6e}")

        # safety small pause (already env.step), but also let ROS spin
        #ros.sleep(0.001)


except KeyboardInterrupt:
    print("MMC loop interrupted by user (KeyboardInterrupt).")

finally:
    # --- Plot manipulability vs time ---
    if len(time_log) > 0:
        plt.figure(figsize=(6,4))
        plt.plot(time_log, manip_log)  # do not set colors per style rules
        plt.xlabel("Time (s)")
        plt.ylabel("Manipulability (prod singular values)")
        plt.title("Manipulability vs Time")
        plt.grid(True)
        plt.tight_layout()
        png_path = "manipulability_vs_time.png"
        plt.savefig(png_path)
        print(f"Saved manipulability plot to {png_path}")
        try:
            plt.show()
        except Exception:
            # non-interactive backend may error on show; it's okay since file is saved
            pass
    else:
        print("No manipulability data recorded; no plot created.")

    print("Finished MMC run.")
    time.sleep(2)
    env.close()
    
# After loop (save recorded joints)
if len(q_log) > 0:
    q_array = np.vstack(q_log)   # shape (N,7)
    csv_path = "joint_angles_mmc.csv"
    np.savetxt(csv_path, q_array, delimiter=',')
    print(f"Saved recorded joint trajectory to {csv_path} (N={q_array.shape[0]} points, dt={dt}s)")
else:
    print("No joint data recorded; CSV not saved.")


confirm = input("Do you want to move the real robot with this trajectory? [y/n]: ").strip().lower()
if confirm == 'y':
    os.system("rosrun panda_move_pkg follow_traj.py --traj_file joint_angles_mmc.csv")
elif confirm == 'n':
    print("Exiting without moving robot.")
else:
    print("Invalid input. Exiting.")
