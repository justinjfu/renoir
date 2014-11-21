"""
Maintains a global "Robot" variable that
subscribes to various topics and records their values
"""
import sys
import rospy
import numpy as np
import moveit_commander

class RobotState(object):
    def __init__(self):
        self.__subscribe()
        self.__publish()
        self.is_hand_down = False
        self.is_moveit_initialized = False

    def __init_moveit(self):
	"""
	"""
    	if (self.is_moveit_initialized):
            return

        #Initialize moveit_commander
    	moveit_commander.roscpp_initialize(sys.argv)

    	#Start a node
    	rospy.init_node('moveit_node')
    
    	#Set up the left gripper
    	left_gripper = baxter_gripper.Gripper('left')
    
    	#Calibrate the gripper
    	print('Calibrating...')
    	left_gripper.calibrate()
    	rospy.sleep(2.0)
	
    	#Initialize left arm
    	self.robot = moveit_commander.RobotCommander()
    	self.scene = moveit_commander.PlanningSceneInterface()
    	self.left_arm = moveit_commander.MoveGroupCommander('left_arm')
    	self.left_arm.set_planner_id('RRTConnectkConfigDefault')
    	self.left_arm.set_planning_time(10)

        #Set flag that its initialized
        self.is_moveit_initialized = True

    def __subscribe(self):
        """
        Initialize all subscribers
        """
        self.pic2world_sub = rospy.Subscriber('renoir/pic2world', lambda msg: self.__pic2world_callback(self))

    def __publish(self):
        """
        Initialize all publishers.
        """
        # Create DisplayTrajectory publishes trajectories for RVIZ
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory)
        # wait for RVIZ to visualize
        print "============ Waiting for RVIZ..."
        rospy.sleep(10)
        print "============ RVIZ started"

    def __pic2world_callback(self, msg):
        self.pic2world_transform = msg.data.reshape((3,3))

    def getPic2World(self):
        """
        Return the latest picture to world frame transformation matrix.
        This is a 3x2 matrix which brings picture frame coordinates (2D) to world
        frame coordinates in Baxter's base frame
        """
        return self.pic2world_transform

    def set_hand_down():
        self.is_hand_down = True

    def set_hand_up():
        self.is_hand_down = False

ROBOT_STATE = RobotState()