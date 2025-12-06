#!/usr/bin/env python3

import sys
import rospy as ros

from actionlib import SimpleActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, \
                             FollowJointTrajectoryGoal, FollowJointTrajectoryResult
import pickle
import argparse
import time
import numpy as np 



if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--traj_file', help='File to load and run')
    args = parser.parse_args()

    ros.init_node('follow_trajectory')
    time.sleep(1)

    # action = ros.resolve_name('~follow_joint_trajectory')
    action = '/effort_joint_trajectory_controller/follow_joint_trajectory'
    client = SimpleActionClient(action, FollowJointTrajectoryAction)
    ros.loginfo("move_to_start: Waiting for '" + action + "' action to come up")
    client.wait_for_server()

    # Open saved trajectory
    # with open(args.traj_file, 'rb') as f:
        # data = pickle.load(f)
        # planned_path = data['robot_traj']
    print("INSIDE FOLLOW_TRAj")
    planned_path = np.loadtxt(args.traj_file, delimiter=',')
    #planned_path = np.load(args.traj_file, allow_pickle=True)
    # planned_path = np.array([[-1.682     ,  0.864     ,  0.803     , -2.254     ,  0.3343    ,
    #      2.351     , -1.689     ],
    #    [-1.68194539,  0.86227225,  0.80382628, -2.25506463,  0.33443022,
    #      2.35156735, -1.68844451],
    #    [-1.68178366,  0.85708573,  0.80631265, -2.25824005,  0.33482486,
    #      2.35324792, -1.68677346],
    #    [-1.68152186,  0.84843081,  0.81048207, -2.26347089,  0.33549528,
    #      2.35597714, -1.68397305],
    #    [-1.68117314,  0.83629095,  0.81637323, -2.2706649 ,  0.33646054,
    #      2.35964663, -1.68002139],
    #    [-1.68075852,  0.82064168,  0.82404085, -2.27969278,  0.33774808,
    #      2.36410294, -1.67489054],
    #    [-1.68030955,  0.80144926,  0.83355625, -2.29038809,  0.33939485,
    #      2.36914582, -1.66854971],
    #    [-1.67987187,  0.77866887,  0.84500814, -2.30254681,  0.34144919,
    #      2.37452605, -1.66096978],
    #    [-1.67950989,  0.7522423 ,  0.85850367, -2.31592679,  0.34397368,
    #      2.37994287, -1.65212963],
    #    [-1.67931276,  0.72209496,  0.8741699 , -2.3302468 ,  0.34704962,
    #      2.38504103, -1.6420249 ],
    #    [-1.67940166,  0.68813151,  0.89215551, -2.34518524,  0.35078412,
    #      2.38940757, -1.63068002],
    #    [-1.67993858,  0.65022933,  0.91263272, -2.36037835,  0.35532117,
    #      2.39256825, -1.61816488],
    #    [-1.68113649,  0.6082277 ,  0.93579895, -2.37541789,  0.36085934,
    #      2.39398321, -1.60461796],
    #    [-1.68326999,  0.56190881,  0.96187688, -2.38984801,  0.36767996,
    #      2.39304086, -1.59027913],
    #    [-1.68654212,  0.51287409,  0.9900126 , -2.40271081,  0.37584616,
    #      2.38925338, -1.5760599 ],
    #    [-1.69102773,  0.46277018,  1.01919361, -2.41335478,  0.38536991,
    #      2.38252009, -1.56288285],
    #    [-1.69684367,  0.41131613,  1.04939135, -2.42173976,  0.39661942,
    #      2.37272785, -1.55116576],
    #    [-1.70405827,  0.35810121,  1.08050484, -2.42781479,  0.41009427,
    #      2.35971351, -1.54142524],
    #    [-1.71262732,  0.3025212 ,  1.11228225, -2.43151193,  0.42647491,
    #      2.34322724, -1.5343114 ],
    #    [-1.7222474 ,  0.24369598,  1.14414881, -2.43273751,  0.44667362,
    #      2.322878  , -1.53064367],
    #    [-1.73202136,  0.18041969,  1.17482852, -2.43136633,  0.47180762,
    #      2.29807711, -1.53138989],
    #    [-1.73979077,  0.11141971,  1.20162235, -2.42727073,  0.5027586 ,
    #      2.26810889, -1.53734476],
    #    [-1.74151702,  0.03675427,  1.21983298, -2.42048313,  0.53848324,
    #      2.23279851, -1.54790323],
    #    [-1.73321609, -0.03962794,  1.22512515, -2.4115017 ,  0.57359937,
    #      2.19403955, -1.55931489],
    #    [-1.71579552, -0.11128143,  1.21885741, -2.40112952,  0.60105347,
    #      2.15551956, -1.56661019],
    #    [-1.69401762, -0.17487394,  1.20659292, -2.38979665,  0.61825577,
    #      2.11961562, -1.56800369],
    #    [-1.67163939, -0.23060241,  1.19263147, -2.37750383,  0.62647333,
    #      2.08666873, -1.56444077],
    #    [-1.6512539 , -0.27794539,  1.17955598, -2.36466048,  0.62782087,
    #      2.05734371, -1.55772225],
    #    [-1.63403401, -0.31701804,  1.16848903, -2.35187112,  0.62471648,
    #      2.03197144, -1.5496354 ],
    #    [-1.61972891, -0.3494994 ,  1.15937318, -2.33933427,  0.6190194 ,
    #      2.0098679 , -1.5411986 ],
    #    [-1.60796367, -0.37658454,  1.15197754, -2.32726474,  0.61194913,
    #      1.99057418, -1.53300226],
    #    [-1.5983665 , -0.39915183,  1.14604418, -2.31587356,  0.60433026,
    #      1.97378024, -1.52538531],
    #    [-1.59060648, -0.41786519,  1.14133427, -2.30535682,  0.59673579,
    #      1.95927333, -1.51853736],
    #    [-1.58440184, -0.43323592,  1.13764085, -2.29588997,  0.58957231,
    #      1.94690515, -1.51255858],
    #    [-1.5795188 , -0.44566129,  1.13479057, -2.28762528,  0.58313224,
    #      1.93657104, -1.50749585],
    #    [-1.57576775, -0.4554494 ,  1.13264196, -2.28069092,  0.57762701,
    #      1.9281967 , -1.50336523],
    #    [-1.5729992 , -0.4628355 ,  1.13108297, -2.27519091,  0.57320854,
    #      1.92172964, -1.50016628],
    #    [-1.57110032, -0.46799314,  1.13002875, -2.2712055 ,  0.56998334,
    #      1.91713354, -1.49789125],
    #    [-1.56999225, -0.47104149,  1.12941981, -2.2687916 ,  0.56802175,
    #      1.91438469, -1.49653096],
    #    [-1.56962815, -0.47205006,  1.12922079, -2.26798319,  0.56736365,
    #      1.9134698 , -1.49607843]])

    joint_link_names = [
        'panda_joint1',
        'panda_joint2',
        'panda_joint3',
        'panda_joint4',
        'panda_joint5',
        'panda_joint6',
        'panda_joint7',
    ]
    
    # Get current pose
    topic = ros.resolve_name('franka_state_controller/joint_states')
    ros.loginfo("move_to_start: Waiting for message on topic '" + topic + "'")
    joint_state = ros.wait_for_message(topic, JointState)
    initial_pose = joint_state.position

    # Add all trajectory points to the goal trajectory.
    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = joint_link_names
    total_duration = 0.0
    for q_i in planned_path:
        # Find delta_pose from previous pose
        delta_pose = [q_i[j] - initial_pose[j] for j in range(7)]
        max_movement = max((abs(delta_pose[j]) for j in range(7)))

        point = JointTrajectoryPoint()
        # Use either the time to move the furthest joint with 'max_dq' or 500ms,
            # whatever is greater
        interval_time = max(max_movement / ros.get_param('~max_dq', 0.2), 0.2)
        total_duration +=interval_time
        point.time_from_start = ros.Duration.from_sec(
            total_duration
        )

        point.positions = q_i.tolist()
        point.velocities = [dq_i/interval_time for dq_i in delta_pose]

        goal.trajectory.points.append(point)
        # Update initial pose to current waypoint
        initial_pose = q_i
    
    # Set the velocity of the last state to be 0.0
    goal.trajectory.points[-1].velocities = [0.0]*7

    goal.goal_time_tolerance = ros.Duration.from_sec(total_duration)

    ros.loginfo('Sending trajectory Goal to move to a current config')
    client.send_goal_and_wait(goal)

    result = client.get_result()
    if result.error_code != FollowJointTrajectoryResult.SUCCESSFUL:
        ros.logerr('move_to_start: Movement was not successful: ' + {
            FollowJointTrajectoryResult.INVALID_GOAL:
            """
            The joint pose you want to move to is invalid (e.g. unreachable, singularity...).
            Is the 'joint_pose' reachable?
            """,

            FollowJointTrajectoryResult.INVALID_JOINTS:
            """
            The joint pose you specified is for different joints than the joint trajectory controller
            is claiming. Does you 'joint_pose' include all 7 joints of the robot?
            """,

            FollowJointTrajectoryResult.PATH_TOLERANCE_VIOLATED:
            """
            During the motion the robot deviated from the planned path too much. Is something blocking
            the robot?
            """,

            FollowJointTrajectoryResult.GOAL_TOLERANCE_VIOLATED:
            """
            After the motion the robot deviated from the desired goal pose too much. Probably the robot
            didn't reach the joint_pose properly
            """,
        }[result.error_code])
    else:
        ros.loginfo('Successfully moved into target pose')
