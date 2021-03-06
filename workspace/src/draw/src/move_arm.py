import moveit_commander
from robot_state import ROBOT_STATE
from moveit_msgs.msg import OrientationConstraint, Constraints
from geometry_msgs.msg import PoseStamped
import sys
import rospy
import numpy

#testing?
def main():
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
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    left_arm = moveit_commander.MoveGroupCommander('left_arm')
    left_arm.set_planner_id('RRTConnectkConfigDefault')
    left_arm.set_planning_time(10)

    test1 = numpy.array([0.2,0.6,0.2])
    test2 = numpy.array([0.6,0.4,0])
    move_arm(test1,test2)
#end?

def move_arm(pos1,pos2):
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

    #goal pose -----------------------------------------------------
    rospy.sleep(2.0)  
    goal_position = PoseStamped()
    goal_position.header.frame_id = "base"

    #x, y, and z position
    goal_position.pose.position.x = pos2[0]
    goal_position.pose.position.y = pos2[1]
    goal_position.pose.position.z = pos2[2]
    
    #Orientation as a quaternion
    goal_position.pose.orientation.x = 0.0
    goal_position.pose.orientation.y = -1.0
    goal_position.pose.orientation.z = 0.0
    goal_position.pose.orientation.w = 0.0

    #Set the goal state to the pose you just defined
    ROBOT_STATE.left_arm.set_pose_target(goal_position)

    #Set the start state for the left arm
    ROBOT_STATE.left_arm.set_start_state_to_current_state()

    # #Create a path constraint for the arm
    # #UNCOMMENT TO ENABLE ORIENTATION CONSTRAINTS
    orien_const = OrientationConstraint()
    orien_const.link_name = "left_gripper";
    orien_const.header.frame_id = "base";
    orien_const.orientation.y = -1.0;
    orien_const.absolute_x_axis_tolerance = 0.1;
    orien_const.absolute_y_axis_tolerance = 0.1;
    orien_const.absolute_z_axis_tolerance = 0.1;
    orien_const.weight = 1.0;
    consts = Constraints()
    consts.orientation_constraints = [orien_const]
    ROBOT_STATE.left_arm.set_path_constraints(consts)

    #Plan a path
    left_plan = ROBOT_STATE.left_arm.plan()

    #Execute the plan
    ROBOT_STATE.left_arm.execute(left_plan)

