#!/usr/bin/env python

from robot_state import ROBOT_STATE
import numpy
import rospy

def draw_line(line_segment):
    """
    Make baxter draw a line segment
     
    :param line_segment: A list of 2 points. Ex. [[x1, y1], [x2, y2]]
    """
    assert ROBOT_STATE.is_hand_down  #Make sure robot hand is down
    assert len(line_segment) == 2

    start = line_segment[0]
    end = line_segment[1]

    #TODO interpolate between start and end
    interpolated = []

    #TODO Call pic2world on each interpolated point as baxter draws
    #We do it here because
    #Ex. pic2world(interpolated[i])


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
    homog = transform.dot(homog)
    result = np.array([homog[0]/homog[2], homog[1]/homog[2]])
    return result

def pic2world_list(point_list):
    """
    Convert a list of picture frame coordinates ( a 2-element numpy array)
    to a 3D world coordinate
    """
    new_list = [None]*len(point_list)
    for i in range(len(point_list)):
        new_list[i] = pic2world(point_list[i])
    return new_list


