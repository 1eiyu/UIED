import math
import sys

import cv2
import rospy
import moveit_commander
import geometry_msgs.msg
import numpy as np
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import tf.transformations as tf
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_matrix
from scipy.spatial.transform import Rotation as R
from ur_dashboard_msgs.srv import Trigger,TriggerResponse
import actionlib
import roslib
from ctl_interface_msgs.msg import PressScreenCoordinateAction 
from ctl_interface_msgs.msg import PressScreenCoordinateGoal


class CtlInterface:

    def __init__(self,group_name='manipulator',mock_interface=False):
        #initialize node
        rospy.init_node("ctl_interface",anonymous=True)
        if not(mock_interface):
            moveit_commander.roscpp_initialize(sys.argv)
            
            #Define Interface
            self.robot = moveit_commander.RobotCommander()
            self.scene = moveit_commander.PlanningSceneInterface()
            #define Movegroup
            self.move_group = moveit_commander.MoveGroupCommander(group_name)
            #sleep for the planing scene to be initialized(?)
            rospy.sleep(1)

            group_names = self.robot.get_group_names()
            # We can get the name of the reference frame for this robot:
            #planning_frame = self.move_group.get_planning_frame()
            # We can also print the name of the end-effector link for this group:
            self.eef_link = self.move_group.get_end_effector_link()
            #ret = self.move_group.set_end_effector_link("endeffector")
            # We can get a list of all the groups in the robot:
            #group_names = self.robot.get_group_names()
            self.poselist=[]

            p = self.move_group.get_current_pose().pose

            # add the trigger server
            s = rospy.Service('trigger_service',Trigger,self.handle_trigger)

        self.actionserver = actionlib.SimpleActionServer('press_screen_coordinate',PressScreenCoordinateAction,self.as_press_execute,False)

        self.actionserver.start()


        #define the variables for the Screen specific part:
        self.normal = None
        self.screen_location = None
        self.screen_x = None
        self.screen_y = None
        self.screen_orientation = None
        self.waiting = False
        self.bottomleftpoint = False

    def as_press_execute(self,goal:PressScreenCoordinateGoal):
        print("action requested:")
        goal.x_coordinate_relative
        print(goal.x_coordinate_relative)
        print(goal)
        x_rel = goal.x_coordinate_relative
        y_rel = goal.y_coordinate_relative
        if 0 <= x_rel <= 1 and 0 <= y_rel <=1:
            print("parameter are in intervall")
            rv = self.touch_point_relative(x_rel,y_rel)
            print("executed with rv: ",rv)
        self.actionserver.set_succeeded()

    def define_edge_points(self):
        """Defines the display by saving the endeffector pose manual calibration
        Args:
        Returns:
        """
        print("Move the endeffector to the top left corner of the screen. Press any key to confirm")
        input("")
        p = self.move_group.get_current_pose().pose
        self.pos1 = np.array([p.position.x,p.position.y,p.position.z])
        print("saved base point:")

        print("Move the endeffector to the top right corner of the screen. Press any key to confirm")
        input("")
        p = self.move_group.get_current_pose().pose
        self.pos2 = np.array([p.position.x,p.position.y,p.position.z])
        print("saved x point:")

        print("Move the endeffector to the bottom left corner of the screen. Press any key to confirm")
        input("")
        p = self.move_group.get_current_pose().pose
        self.pos3 = np.array([p.position.x,p.position.y,p.position.z])
        print("saved y point:")

        self.define_screen(self.pos1,self.pos2-self.pos1,self.pos3-self.pos1)
        self.add_display_to_planningscene(self.screen_orientation)


    def init_planingscene(self):
        """Builds a floor object at the base level of the robot with defined size
        Args:
        Returns:
        """
        
        #create a floor with 1m scale
        p = geometry_msgs.msg.PoseStamped()
        p.header.frame_id = self.robot.get_planning_frame()
        p.pose.position.x = 0
        p.pose.position.y = 0
        p.pose.position.z = 0
        p.pose.orientation.w = 1.0
        box_name = "floor"
        self.scene.add_box(box_name,p,size = (1,1,0))

    def add_display_to_planningscene(self,orientation,axes='sxyz'):
        """Computes the Pose of the camera looking at a screen defined by 4 points
        Args:
            orientation ((3,) numpy array): vecor with rotation angles [alphy,beta,gamma]
            axes (string): rotation scheme
        Returns:
        """
        
        p = geometry_msgs.msg.PoseStamped()
        p.header.frame_id = self.robot.get_planning_frame()
        #get the middle point of the screen
        middle = self.screen_location + self.screen_x/2 + self.screen_y/2+0.005*self.normal
        p.pose = self.get_pose(middle,orientation,axes,zrot=0)
        box_name = "screen"
        self.scene.add_box(box_name,p,size = (np.linalg.norm(self.screen_x),np.linalg.norm(self.screen_y),0))

    def get_screenpose(self,axes='sxyz'):
        """Returns the screenpose of the display
        Args:
            axes (string): rotation scheme
        Returns:
            orientation ((3,) numpy array):  the rotation angles
        """
        rotationmatrix = np.hstack([self.screen_x.reshape(-1,1)/np.linalg.norm(self.screen_x),self.screen_y.reshape(-1,1)/np.linalg.norm(self.screen_y),self.normal.reshape(-1,1)])
        al,be,ga = euler_from_matrix(rotationmatrix,'sxyz')
        return np.array([al,be,ga])

    def add_camera_to_planningscene(self,t,rot_matrix,axes='sxyz'):
        """Returns the screenpose of the display
        Args:
            t (3x1 np.array): translation vector
            rot_matrix (3x3 np.array): rotation matrix
            axes (string): rotation scheme
        Returns:
            orientation ((3,) numpy array):  the rotation angles
        """
        p = geometry_msgs.msg.PoseStamped()
        p.header.frame_id = self.robot.get_planning_frame()
        #calculate the linear transform based on the rotation matrix and translation vector t
        rot_matrix_screen = np.hstack([self.screen_x.reshape(-1,1)/np.linalg.norm(self.screen_x),self.screen_y.reshape(-1,1)/np.linalg.norm(self.screen_y),self.normal.reshape(-1,1)])
        rot_combined = np.dot(rot_matrix,rot_matrix_screen)
        location = self.bottomleftpoint + np.dot(rot_matrix_screen,t.reshape(-1,))
        al,be,ga = euler_from_matrix(rot_combined,axes)
        p.pose = self.get_pose(location.reshape((-1)),[al,be,ga],axes,zrot=0)
        self.scene.add_cylinder("camera", p, 0.03, 0.015)

    def get_pose(self,pos,angles,axes = 'sxyz',zrot = 0):
        """Returns the Pose object for building trajectories in ROS
        Args:
            pos ((3,) np.array): position vector (robot coordinate system)
            angles ((3,) np.array):  the rotation angles
        Returns:
            pose (geometry_msgs.msg.Pose):  Pose
        """
        if(zrot!=0):
            q = quaternion_from_euler(0,0,zrot,axes)
            q = q*quaternion_from_euler(angles[0],angles[1],angles[2],axes)
        else:
            q = quaternion_from_euler(angles[0],angles[1],angles[2],axes)
        #q = quaternion_from_euler(0,0,zrot)*q

        return geometry_msgs.msg.Pose(geometry_msgs.msg.Vector3(pos[0],pos[1],pos[2]),geometry_msgs.msg.Quaternion(q[0],q[1],q[2],q[3]))

    def define_screen(self,location,x,y):
        """Defines thes screen location with the given parameter
        Args:
            location ((3,1) np.array): location of the top left coordinate
            x ((3,1) np.array): x vector of the screen
            y((3,1) np.array): y vector of the screen
        Returns:
        """
        #defines the screen with x and y vector
        self.screen_location = location
        self.screen_x = x
        self.screen_y = y
        self.bottomleftpoint = location + y
        normal = np.cross(x,y)
        self.normal = normal/np.linalg.norm(normal)
        #old one
        #self.screen_orientation = self.get_tcp_rotation(normal)
        #new one
        self.screen_orientation = self.get_screenpose()
        #correct the rotation of zhe z axis
        r = R.from_rotvec(self.screen_orientation)
        x_rotated = np.dot(np.array([1,0,0]),r.as_matrix())
        #get the angle between the rotated x axis and the screen x axis
        rot_z = np.arccos(np.dot(x_rotated,self.screen_x))
        #self.screen_orientation[2] = rot_z


    
    def get_tcp_rotation(self,normal,rot_z = 0):
        """get the rotation of the tool center point depending on the normal of the screen
        Args:
            normal ((3,) np.array): vector of the tool center point
            rot_z (float): rotation of the z axis
        Returns:
        """
        beta = math.atan(-normal[0]/normal[2])
        alpha = math.atan(normal[1]/(math.cos(beta)*normal[2]-math.sin(beta)*normal[0]))

        if normal[2]<0:
            alpha = -alpha
            beta = math.pi + beta

        
        return [-alpha,-beta,rot_z]
    
    def get_screen_rotation(self):
        return self.screen_orientation

    def add_pose_list(self,poses):
        self.poselist = self.poselist + poses

    def clear_pose_list(self):
        self.poselist = []
    
    def plan_and_execute(self):
        """Plans and executes the current poselist
        Args:
        Returns:
        """
        self.move_group.set_pose_targets(self.poselist)
        plan = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

    def plan_and_execute_cartesian(self):
        """Plans and executes the catesian path plan given by the current poselist
        Args:
        Returns:
        """
        (plan,fraction) = self.move_group.compute_cartesian_path(self.poselist,0.0001,5)#0,01,5
        #plan = self.move_group.retime_trajectory(self.robot.get_current_state(), plan,velocity_scaling_factor = 0.1,acceleration_scaling_factor = 0.2)		
        print(plan.joint_trajectory.points[-1].time_from_start.to_sec())
        rv = self.move_group.execute(plan, wait=True)
        print(rv)
        return rv

    def touch_point_trajectory(self,point,angle,normal,distance = 0.01,touch_depth = 0.001):
        """Builds trajectory for touching a specified point
        Args:
            point ((3,) np.array): point in 3d space
            angle ((3,) np.array): rotation of the tool center point
            normal ((3,) np.array): normal vector of the screen plane
            distance (float): start and end distance normal to the screen in m
            touch depth (float): depth of the touch in m (depending on the endeffector and the tip) 
        Returns:
            traj (list): trajectory list of the pose goals
        """
        traj = [
            self.get_pose(point-(distance*normal),angle),
            self.get_pose(point+(touch_depth*normal),angle),
            self.get_pose(point-(distance*normal*0.1),angle)
        ]
        return traj
    
    def drag_trajectory(self,point1,point2,angle,normal,distance = 0.005,touch_depth = 0.001):
        """Builds a trajectory for draging from point1 to point 2
        Args:
            point1 ((3,) np.array): start-point in 3d space
            point2 ((3,) np.array): end-point in 3d space
            angle ((3,) np.array): rotation of the tool center point
            normal ((3,) np.array): normal vector of the screen plane
            distance (float): start and end distance normal to the screen in m
            touch depth (float): depth of the touch in m (depending on the endeffector and the tip) 
        Returns:
            traj (list): trajectory list of the pose goals
        """
        traj = [
            self.get_pose(point1-(distance*normal),angle),
            self.get_pose(point1 + touch_depth * normal,angle),
            self.get_pose(point2 + touch_depth * normal,angle),
            self.get_pose(point2-(distance*normal),angle)
        ]
        return traj

    def touch_point_relative(self,x_rel,y_rel):
        """Builds and executes a touch trajectory points relative to the screen dimensions and an normal angle of the tcp
        Args:
            x_rel (float): x coordinate relative to the screen dimensions [0-1]
            y_rel (float): y coordinate relative to the screen dimensions [0-1]
        Returns:
            traj (list): trajectory list of the pose goals
        """
        point = self.screen_location + x_rel * self.screen_x + y_rel* self.screen_y
        trajectory = self.touch_point_trajectory(point,self.screen_orientation,self.normal)
        #trajectory.append(self.get_pose(point + 0.001*self.normal,self.screen_orientation))
        self.add_pose_list(trajectory)
        rv = self.plan_and_execute_cartesian()
        self.clear_pose_list()
        return rv

    def slide_relative(self,x1_rel,y1_rel,x2_rel,y2_rel):
        """Builds and executes a touch trajectory points relative to the screen dimensions and an normal angle of the tcp
        Args:
            x1_rel (float): x coordinate of the startpoint relative to the screen dimensions [0-1]
            y1_rel (float): y coordinate of the startpoint relative to the screen dimensions [0-1]
            x2_rel (float): x coordinate of the endpoint relative to the screen dimensions [0-1]
            y2_rel (float): y coordinate of the endpoint relative to the screen dimensions [0-1]
        Returns:
            traj (list): trajectory list of the pose goals
        """
        point1 = self.screen_location + x1_rel * self.screen_x + y1_rel * self.screen_y
        point2 = self.screen_location + x2_rel * self.screen_x + y2_rel * self.screen_y
        trajectory = self.drag_trajectory(point1,point2,self.screen_orientation,self.normal)
        self.add_pose_list(trajectory)
        self.plan_and_execute_cartesian()
        self.clear_pose_list()
    
    def wait_for_trigger(self):
        """enters a state where the ctl_interface waits for an external trigger
        Args:
        Returns:
        """
        self.waiting = True
        while(self.waiting):
            rospy.sleep(0.1)
    
    def time_trigger(self,duration):
        """sleeps for a duration in seconds
        Args:
        Returns:
        """
        rospy.sleep(duration)

    def handle_trigger(self,req):
        """service routine that is entered when the trigger service has been called
        Args:
        Returns:
        """
        if self.waiting:
            self.waiting = False
            return TriggerResponse(True)
        else:
            return TriggerResponse(False)

if __name__ == '__main__':
    ctl_interface = CtlInterface(mock_interface=True)

