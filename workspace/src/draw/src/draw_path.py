#!/usr/bin/env python

import sys
import copy
import math
import numpy
import rospy
from robot_state import ROBOT_STATE
import moveit_commander
from robot_state import ROBOT_STATE
from moveit_msgs.msg import OrientationConstraint, Constraints
from geometry_msgs.msg import PoseStamped

def draw_line(line_segment):
    """
    Make baxter draw a line segment
     
    :param line_segment: A list of 2 points. Ex. [[x1, y1], [x2, y2]]
    """
    assert ROBOT_STATE.is_hand_down  #Make sure robot hand is down
    assert len(line_segment) == 2

    wstart = pic2world(line_segment[0]) #Start and end coordinates in world frame 
    wend = pic2world(line_segment[1])

    # Initialize moveit if not already initialized
    if (!ROBOT_STATE.is_moveit_initialized):
        ROBOT_STATE.__init_moveit()

    move_to_start(world_start)

    # List of waypoints for the end-effector to go through. Use to plan cartesian path.
    waypoints = []
    # Add start point
    waypoints.append(ROBOT_STATE.left_arm.get_current_pose().pose)

    # 
    x_dist = wend[0] - wstart[0]
    y_dist = wend[1] - wstart[1]
    z_dist = wend[2] - wstart[2]
    euclid_dist = math.sqrt(x_dist**2 + y_dist**2 + z_dist**2)
    num_waypoints = euclid_dist/0.1
    dx = x_dist/num_waypoints
    dy = y_dist/num_waypoints
    dz = z_dist/num_waypoints

    wpose = geometry_msgs.msg.Pose()
    wpose.orientation.w = 1.0
    wpose.position.x = waypoints[0].position.x
    wpose.position.y = waypoints[0].position.y
    wpose.position.z = waypoints[0].position.z
    for i in range(0, int(num_waypoints)):
        wpose.position.x += dx
        wpose.position.y += dy
        wpose.position.z += dz
        waypoints.append(copy.deepcopy(wpose))

    wpose.position.x = wend[0]
    wpose.position.y = wend[1]
    wpose.position.z = wend[2]

    # Add end point
    (plan3, fraction) = ROBOT_STATE.left_arm.compute_cartesian_path(waypoints, 0.1, 0.0)

def move_to_start(start):
    #Start pose ------------------------------------------------------
    start_position = PoseStamped()
    start_position.header.frame_id = "base"

    #x, y, and z position
    start_position.pose.position.x = pos1[0]
    start_position.pose.position.y = pos1[1]
    start_position.pose.position.z = pos1[2]
    
    #Orientation as a quaternion
    start_position.pose.orientation.x = 0.0
    start_position.pose.orientation.y = -1.0
    start_position.pose.orientation.z = 0.0
    start_position.pose.orientation.w = 0.0

    #Set the goal state to the pose you just defined
    ROBOT_STATE.left_arm.set_pose_target(start_position)

    #Set the start state for the left arm
    ROBOT_STATE.left_arm.set_start_state_to_current_state()

    #Plan a path
    left_plan = ROBOT_STATE.left_arm.plan()

    #Execute the plan
    ROBOT_STATE.left_arm.execute(left_plan)

def bring_up():
    """
    raise hand 2 inches from current point
    """
    #TODO(Hong): Bring hand up to a fixed position via MoveIt
    
    ROBOT_STATE.set_hand_up()

def bring_down(target_point):
    """
    move hand to initial point to start drawing from

    :param target_point: A 2D image frame point [x1, y1]
    """
    if ROBOT_STATE.is_hand_down:
        raise RuntimeException("Robot hand already down! Need to call bring_up first")

    #TODO(Hong): Bring hand down via MoveIt
    target_point = pic2world(target_point)

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
    return transform.dot(homog)

def pic2world_list(point_list):
    """
    Convert a list of picture frame coordinates ( a 2-element numpy array)
    to a 3D world coordinate
    """
    new_list = [None]*len(point_list)
    for i in range(len(point_list)):
        new_list[i] = pic2world(point_list[i])
    return new_list


