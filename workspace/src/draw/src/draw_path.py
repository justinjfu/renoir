#!/usr/bin/env python

from robot_state import ROBOT_STATE
import numpy
import rospy

# use moveit to plan and move baxter arm
def draw_line(linesegment):
    pass

# raise hand 2 inches from current point
def bring_up():
    #Bring hand up to a fixed position via MoveIt
    
    ROBOT_STATE.set_hand_up()

# move hand to initial point to start drawing from
def bring_down(target_point):
    if ROBOT_STATE.is_hand_down:
        raise RuntimeException("Robot hand already down! Need to call bring_up first")

    #Bring hand down via MoveIt

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
    return transform.dot(single_point)

def pic2world_list(point_list):
    """
    Convert a list of picture frame coordinates ( a 2-element numpy array)
    to a 3D world coordinate
    """
    new_list = [None]*len(point_list)
    for i in range(len(point_list)):
        new_list[i] = pic2world(point_list[i])
    return new_list


