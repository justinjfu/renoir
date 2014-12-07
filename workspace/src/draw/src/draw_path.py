#!/usr/bin/env python

import sys
import copy
import math
import numpy as np
import rospy
from robot_state import ROBOT_STATE
import moveit_commander
from moveit_msgs.msg import OrientationConstraint, Constraints
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped

def draw_line(line_segment):
    """
    Make baxter draw a line segment
     
    :param line_segment: A list of 2 points. Ex. [[x1, y1], [x2, y2]]
    """
    assert ROBOT_STATE.is_hand_down  #Make sure robot hand is down
    assert len(line_segment) == 2

    # wstart = pic2world(line_segment[0]) #Start and end coordinates in world frame 
    # wend = pic2world(line_segment[1])
    wstart = line_segment[0]
    wend = line_segment[1]
    
    # List of waypoints for the end-effector to go through. Use to plan cartesian path.
    waypoints = []
    
    # Add start point
    wpose = PoseStamped()
    wpose.header.frame_id = "base"
    wpose.pose.position.x = wstart[0]
    wpose.pose.position.y = wstart[1]
    wpose.pose.position.z = wstart[2]
    wpose.pose.orientation.x = ROBOT_STATE.orientation[0]
    wpose.pose.orientation.y = ROBOT_STATE.orientation[1]
    wpose.pose.orientation.z = ROBOT_STATE.orientation[2]
    wpose.pose.orientation.w = ROBOT_STATE.orientation[3]
    waypoints.append(copy.deepcopy(wpose.pose))

    # Add end point
    wpose.pose.position.x = wend[0]
    wpose.pose.position.y = wend[1]
    wpose.pose.position.z = wend[2]
    waypoints.append(copy.deepcopy(wpose.pose))

    print "Executing draw_line"
    (plan3, fraction) = ROBOT_STATE.left_arm.compute_cartesian_path(waypoints, 0.01, 0.0)
    ROBOT_STATE.left_arm.execute(plan3)
    ROBOT_STATE.position = wend

def draw_waypoints(line_segments):
    """
    Make baxter draw a line segment
     
    :param line_segment: A list of points to draw through. Ex. [[x1, y1], [x2, y2], [x3,y3, ...]]
    """
    assert ROBOT_STATE.is_hand_down  #Make sure robot hand is down

    # List of waypoints for the end-effector to go through. Use to plan cartesian path.
    waypoints = []
    
    # Add start point
    wpose = PoseStamped()
    wpose.header.frame_id = "base"
    wpose.pose.orientation.x = ROBOT_STATE.orientation[0]
    wpose.pose.orientation.y = ROBOT_STATE.orientation[1]
    wpose.pose.orientation.z = ROBOT_STATE.orientation[2]
    wpose.pose.orientation.w = ROBOT_STATE.orientation[3]

    for i in range(len(line_segments)):
        line_segments[i] = pic2world(line_segments[i])

    for wpoint in line_segments:
        wpose.pose.position.x = wpoint[0]
        wpose.pose.position.y = wpoint[1]
        wpose.pose.position.z = wpoint[2]
        waypoints.append(copy.deepcopy(wpose.pose))

    print "Executing draw_waypoints"
    (plan3, fraction) = ROBOT_STATE.left_arm.compute_cartesian_path(waypoints, 0.01, 0.0)
    ROBOT_STATE.left_arm.execute(plan3)
    ROBOT_STATE.position = wend

def bring_up():
    """
    raise hand 2 inches from current point
    """
    # Bring hand up 5 centimeters
    if not ROBOT_STATE.is_hand_down:
        raise RuntimeException("Robot hand already up!")
    
    current_pose = geometry_msgs.msg.Pose()
    current_pose = ROBOT_STATE.left_arm.get_current_pose().pose
    print "current_pose"
    print current_pose.position
    print current_pose.orientation
    
    desired_pose = PoseStamped()
    desired_pose.header.frame_id = "base"

    desired_pose.pose.position.x = ROBOT_STATE.position[0]
    desired_pose.pose.position.y = ROBOT_STATE.position[1]
    desired_pose.pose.position.z = ROBOT_STATE.position[2] + 0.2
    
    #Orientation as a quaternion
    desired_pose.pose.orientation.x = ROBOT_STATE.orientation[0]
    desired_pose.pose.orientation.y = ROBOT_STATE.orientation[1]
    desired_pose.pose.orientation.z = ROBOT_STATE.orientation[2]
    desired_pose.pose.orientation.w = ROBOT_STATE.orientation[3]

    print "desired_pose"
    print desired_pose.pose.position
    print desired_pose.pose.orientation
    #Set the goal state to the pose you just defined
    ROBOT_STATE.left_arm.set_pose_target(desired_pose)

    #Set the start state for the left arm
    ROBOT_STATE.left_arm.set_start_state_to_current_state()

    #Plan a path
    up_plan = ROBOT_STATE.left_arm.plan()
    # import pdb
    # pdb.set_trace()

    #Execute the plan
    print "Executing bring_up"
    ROBOT_STATE.left_arm.execute(up_plan) #Not sure why but the desired pose is all the way extended

    ROBOT_STATE.position[2] += 0.2
    ROBOT_STATE.set_hand_up()
    print desired_pose.pose.position
    print desired_pose.pose.orientation

def bring_down_world(world_point):
    """
    move hand to initial point to start drawing from

    :param target_point: A 2D image frame point [x1, y1]
    """
    # if ROBOT_STATE.is_hand_down:
    #     raise RuntimeException("Robot hand already down! Need to call bring_up first")

    # # Bring hand down via MoveIt
    # target_point = pic2world(target_point)

    #Start pose ------------------------------------------------------
    start_position = PoseStamped()
    start_position.header.frame_id = "base"

    #x, y, and z position
    start_position.pose.position.x = world_point[0]
    start_position.pose.position.y = world_point[1]
    start_position.pose.position.z = world_point[2]
    
    #Orientation as a quaternion
    start_position.pose.orientation.x = ROBOT_STATE.orientation[0]
    start_position.pose.orientation.y = ROBOT_STATE.orientation[1]
    start_position.pose.orientation.z = ROBOT_STATE.orientation[2]
    start_position.pose.orientation.w = ROBOT_STATE.orientation[3]
    #Set the goal state to the pose you just defined
    ROBOT_STATE.left_arm.set_pose_target(start_position)

    #Set the start state for the left arm
    ROBOT_STATE.left_arm.set_start_state_to_current_state()

    #Plan a path
    left_plan = ROBOT_STATE.left_arm.plan()

    #Execute the plan
    print "Executing bring_down_world"
    ROBOT_STATE.left_arm.execute(left_plan)

    ROBOT_STATE.position = world_point

    ROBOT_STATE.set_hand_down()


def in_order_plan(line_segments):
    """
    Line planning algorithm. Takes a list of line segments and
    issues drawing commands.
    Currently draws them in order
    """
    if not line_segments:
        #If empty list, do nothing
        return

    #a line segment consists of [x1,y1],[x2,y2]
    for i in range(len(line_segments)):
        segment = line_segments[i]
        bring_down(segment[0])
        draw_line(segment)
        bring_up()

def pic2world(single_point):
    """
    Convert a single picture frame coordinate ( a 2-element numpy array)
    to a 3D world coordinate
    """
    transform = ROBOT_STATE.getPic2World()
    homog = np.array([single_point[0], single_point[1], 1])
    result = transform.dot(homog)
    result[2] += 0.05
    return result

def pic2world_list(point_list):
    """
    Convert a list of picture frame coordinates ( a 2-element numpy array)
    to a 3D world coordinate
    """
    return [pic2world(point) for point in point_list]

