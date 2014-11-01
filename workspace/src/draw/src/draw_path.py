#!/usr/bin/env python

from robot_state import ROBOT_STATE
import numpy
import rospy

# use moveit to plan and move baxter arm
def draw_line(linesegment):

# raise hand 2 inches from current point
def bring_up():

# move hand to initial point to start drawing from
def bring_down(target_point):

# planner, takes in list of line segments. Tells baxter which ones to draw
def plan_line_order(line_segments):


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


