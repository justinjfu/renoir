
THIS_FILE_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$THIS_FILE_DIR/workspace

alias gravity='rosrun baxter_tools enable_robot.py -e'
