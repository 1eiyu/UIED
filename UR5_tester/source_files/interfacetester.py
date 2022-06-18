from ctl_interface.Ctl_interface import CtlInterface
#from source_files.ctl_interface import Ctl_interface
import rospy

from ctl_interface import CtlInterface

import actionlib
import actionlib_tutorials.msg
import ctl_interface_msgs.msg

def clickClient(x_rel,y_rel):

    client = actionlib.SimpleActionClient('press_screen_coordinate',ctl_interface_msgs.msg.PressScreenCoordinateAction)

    client.wait_for_server()

    goal = ctl_interface_msgs.msg.PressScreenCoordinateGoal(x_coordinate_relative=x_rel,y_coordinate_relative=y_rel)

    client.send_goal(goal)

    client.wait_for_result()

    print(client.get_result)

if __name__ == '__main__':
    try:
        rospy.init_node('interface_tester_py')
        clickClient(0.5,0.5)
    except rospy.ROSInterruptException:
        print("program interrupted")