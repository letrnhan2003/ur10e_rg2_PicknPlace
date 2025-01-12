#!/usr/bin/env python

from __future__ import print_function

import rospy
import sys
import copy
import math
import moveit_commander

import moveit_msgs.msg
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint, BoundingVolume
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState
import geometry_msgs.msg
from geometry_msgs.msg import Quaternion, Pose
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

from ur10e_rg2_moveit.srv import MoverService, MoverServiceRequest, MoverServiceResponse

joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

# Between Melodic and Noetic, the return type of plan() changed. moveit_commander has no __version__ variable, so checking the python version as a proxy
if sys.version_info >= (3, 0):
    def planCompat(plan):
        return plan[1]
else:
    def planCompat(plan):
        return plan

"""
    Given the start angles of the robot, plan a trajectory that ends at the destination pose.
    Retry until a valid trajectory is found or maximum attempts are reached.
"""
def plan_trajectory(move_group, destination_pose, start_joint_angles, max_attempts=5):
    move_group.set_max_velocity_scaling_factor(0.001)
    move_group.set_max_acceleration_scaling_factor(0.001)

    for attempt in range(max_attempts):
        try:
            current_joint_state = JointState()
            current_joint_state.name = joint_names
            current_joint_state.position = start_joint_angles

            moveit_robot_state = RobotState()
            moveit_robot_state.joint_state = current_joint_state
            move_group.set_start_state(moveit_robot_state)

            move_group.set_planner_id("RRTstar")
            move_group.set_planning_time(15.0)

            move_group.set_pose_target(destination_pose)
            plan = move_group.plan()

            if plan and plan[1].joint_trajectory.points:
                return planCompat(plan)

        except Exception as e:
            rospy.logwarn(f"Planning attempt {attempt + 1} failed: {e}")

    raise Exception(f"Trajectory planning failed after {max_attempts} attempts.")

"""
    Strengthen the gripper by sending appropriate commands.
"""
def strengthen_gripper():
    # Assuming the gripper is controlled through a ROS topic or service
    gripper_topic = "/gripper_control"
    try:
        gripper_pub = rospy.Publisher(gripper_topic, String, queue_size=10)
        rospy.sleep(1)  # Ensure the publisher is ready
        gripper_pub.publish("strong_grip")  # Replace with your gripper-specific command
    except Exception as e:
        rospy.logwarn(f"Failed to strengthen gripper: {e}")

"""
    Plan the pick-and-place trajectory.
"""
def plan_pick_and_place(req):
    response = MoverServiceResponse()

    group_name = "arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    current_robot_joint_configuration = req.joints_input.joints

    # Pre grasp
    pre_grasp_pose = plan_trajectory(move_group, req.pick_pose, current_robot_joint_configuration)
    if not pre_grasp_pose.joint_trajectory.points:
        return response

    previous_ending_joint_angles = pre_grasp_pose.joint_trajectory.points[-1].positions

    # Grasp
    pick_pose = copy.deepcopy(req.pick_pose)
    pick_pose.position.z -= 0.05  # Static value from Unity, consider passing dynamically
    grasp_pose = plan_trajectory(move_group, pick_pose, previous_ending_joint_angles)
    if not grasp_pose.joint_trajectory.points:
        return response

    # Strengthen the gripper after grasping
    strengthen_gripper()

    previous_ending_joint_angles = grasp_pose.joint_trajectory.points[-1].positions

    # Pick Up
    pick_up_pose = plan_trajectory(move_group, req.pick_pose, previous_ending_joint_angles)
    if not pick_up_pose.joint_trajectory.points:
        return response

    previous_ending_joint_angles = pick_up_pose.joint_trajectory.points[-1].positions

    # Place
    place_pose = plan_trajectory(move_group, req.place_pose, previous_ending_joint_angles)
    if not place_pose.joint_trajectory.points:
        return response

    # Append all plans to the response
    response.trajectories.append(pre_grasp_pose)
    response.trajectories.append(grasp_pose)
    response.trajectories.append(pick_up_pose)
    response.trajectories.append(place_pose)

    move_group.clear_pose_targets()

    return response

def moveit_server():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('ur10e_rg2_moveit_server')

    s = rospy.Service('ur10e_rg2_moveit', MoverService, plan_pick_and_place)
    print("Ready to plan")
    rospy.spin()

if __name__ == "__main__":
    moveit_server()
