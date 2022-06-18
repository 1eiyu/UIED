#this is the main test skript that runs the guis and the control interface for the robot

from pickle import FALSE
from ctl_interface import CtlInterface
from GUI import corner_select
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import cv2
import numpy as np
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from GUI import Scripter


def save_points_to_file(p1,p2,p3,fname="pos.txt"):
    tosave = np.array([p1,p2,p3])
    np.savetxt(fname,tosave)
    print(tosave)


def read_points_from_file(fname="pos.txt"):
    res = np.fromfile(fname,dtype=float,sep=' ',)
    return res.reshape((3,-1))


def get_camera_pose(A,d,m,width):
    """Computes the Pose of the camera looking at a screen defined by 4 points
    Args:
        A (3x3 np.array): Camera Matrix
        d (1x5 vector): distortion coefficients
        m (4x3 np.array): 4 homogenous coordinates of the 4 edge points
        width: (float): width of the screen in mm 

    Returns:
        R (3x3 np.array): rotation matrix
        t (3x1 np.array): translation vector
    """
    m1 = m[0]
    m2 = m[1]
    m3 = m[2]
    m4 = m[3]

    k2 = np.dot(np.cross(m1,m4),m3)/np.dot(np.cross(m2,m4),m3)
    k3 = np.dot(np.cross(m1,m4),m2)/np.dot(np.cross(m3,m4),m2)

    n2 = k2*m2-m1
    n3 = k3*m3-m1
    r1 = np.dot(np.linalg.inv(A),n2)/np.linalg.norm(np.dot(np.linalg.inv(A),n2))
    r2 = -np.dot(np.linalg.inv(A),n3)/np.linalg.norm(np.dot(np.linalg.inv(A),n3))
    r3 = np.cross(r1,r2)
    r3 = r3/np.linalg.norm(r3)
    lambda1 = width*np.sqrt(1/np.dot(n2.T,np.dot(np.linalg.inv(A.T),np.dot(np.linalg.inv(A),n2))))
    t = lambda1*np.dot(np.linalg.inv(A),m1)
    R = np.hstack([r1.reshape(-1,1),r2.reshape(-1,1),r3.reshape(-1,1)])

    return R,t.reshape(1,3)




if __name__ == "__main__":
    #Set the following parameters if needed
    fromfile = True #load the calibrated edge points from a file ("pos.txt")
    writetofile = True #saves the edge points to file ("pos.txt")

    #camera intrinsics

    #camera matrix
    mtx = np.array([[1.34497367e+03, 0.00000000e+00, 8.93162071e+02],
        [0.00000000e+00, 1.29002470e+03, 6.48903045e+02],
        [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
    
    #distortion coefficients
    dist = np.array([[-0.31259256, -0.23645597, -0.03902126,  0.00549274,  0.97294402]])

    ctl_interface = CtlInterface()
    ctl_interface.init_planingscene()

    if not fromfile:
        ctl_interface.define_edge_points()
        pos1 = ctl_interface.pos1
        pos2 = ctl_interface.pos2
        pos3 = ctl_interface.pos3
        p12 = pos2 - pos1
        p13 = pos3 - pos1
        pedge = pos1+p12+p13
        normal = np.cross(p12,p13)
        normal = normal/np.linalg.norm(normal)
        angles = ctl_interface.get_tcp_rotation(normal)
        if writetofile:
            save_points_to_file(pos1,pos2,pos3)
        

    else:
        #read the calibrated points from file
        (pos1,pos2,pos3) = np.array([[-3.257933086256111777e-01,-5.195390333884183365e-01,3.784189333620610052e-01],[-4.694349160298382229e-01,-3.629499235948669966e-01,3.700038692520515871e-01],[-2.532091873278026761e-01,-4.594788153365423344e-01,2.467866729703509232e-01]])#read_points_from_file()
        #calculate the x,y vectors of the image plane
        p12 = pos2 - pos1
        p13 = pos3 - pos1
        
        #define the screen in the ctl_interface
        ctl_interface.define_screen(pos1,p12,p13)
        
        #add the screen in the planning interface
        ctl_interface.add_display_to_planningscene(ctl_interface.screen_orientation)

        pedge = pos1+p12+p13
        normal = np.cross(p12,p13)
        normal = normal/np.linalg.norm(normal)
        angles = ctl_interface.get_tcp_rotation(normal)

    # go to home positon arbitrary selected
    ctl_interface.add_pose_list([ctl_interface.get_pose(pos1+p12/2+p13/2 - normal*0.06,angles),ctl_interface.get_pose(pos1+p12/2+p13/2 - normal*0.08,angles)])
    ctl_interface.plan_and_execute_cartesian()
    ctl_interface.clear_pose_list()
    
    #set up the video capture device (here it is device 1, because 0 would be the internal webcam)
    cap = cv2.VideoCapture(-1)
    #set up the codec for the camera (depending on camera hardware)
    cap.set(cv2.CAP_PROP_FOURCC, 0x47504A4D)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

    #initialize the plannin scene
    ctl_interface.init_planingscene()

    #dummy values for the corner selection to test camera pose estimation
    m1 = np.array([373,660,1])
    m2 = np.array([1449,605,1])
    m3 = np.array([491,170,1])
    m4 = np.array([1269,144,1])

    R,t = get_camera_pose(mtx,dist,np.array([m1,m2,m3,m4]),0.2145)
    #test the pose estimation
    #ctl_interface.add_camera_to_planningscene(-t,R)
    #rospy.sleep(0.1)

    # start the gui for the corner selection
    coordinatelist = corner_select.select_corner_gui(camera=cap,mtx=mtx,dist=dist)
    #dummy values for test purposes
    #coordinatelist = [(621, 383), (978, 348), (1009, 542), (636, 589)]

    #start the main GUI
    Scripter.UI_Skript_Recorder(cap,coordinatelist,mtx=mtx,dist=dist,ctl_interface=ctl_interface)


    #Some Trajectories just for Test purposes
    
    # trajectory1
    # pose_list = [
    #             ctl_interface.get_pose(pos1- normal*0.005,angles),
    #             ctl_interface.get_pose(pos2- normal*0.005,angles),
    #             ctl_interface.get_pose(pedge- normal*0.005,angles),
    #             ctl_interface.get_pose(pos3- normal*0.005,angles),
    #     ]
    # ctl_interface.add_pose_list(pose_list)
    # for pose in pose_list:
    #     ctl_interface.clear_pose_list()
    #     ctl_interface.add_pose_list([pose])
    #     ctl_interface.plan_and_execute()
    

    #trajectory2
    # pose_list = [ctl_interface.touch_point_trajectory(pos1- normal*0.001,angles,normal), \
    # ctl_interface.touch_point_trajectory(pos2- normal*0.002,angles,normal), \
    # ctl_interface.touch_point_trajectory(pedge- normal*0.002,angles,normal), \
    # ctl_interface.touch_point_trajectory(pos3- normal*0.002,angles,normal)]
    # for traj in pose_list:
    #     ctl_interface.clear_pose_list()
    #     ctl_interface.add_pose_list(traj)
    #     ctl_interface.plan_and_execute_cartesian()

    #trajectory2
    #pose_list = [ctl_interface.touch_point_trajectory(pos1+0.5*p12+0.5*p13 + normal*0.001,angles,normal), \
    #ctl_interface.touch_point_trajectory(pos1+0.7*p12+0.2*p13+ normal*0.001,angles,normal), \
    #ctl_interface.touch_point_trajectory(pos1+0.2*p12+0.7*p13+ normal*0.001,angles,normal), \
    #ctl_interface.touch_point_trajectory(pos1+0.3*p12+0.3*p13+ normal*0.001,angles,normal), \
    #ctl_interface.drag_trajectory(pos1+0.1*p12+0.5*p13 + normal*0.001,pos1+0.75*p12+0.5*p13 + normal*0.001,angles,normal), \
    #ctl_interface.drag_trajectory(pos1+0.8*p12+0.5*p13 + normal*0.001,pos1+0.2*p12+0.5*p13 + normal*0.001,angles,normal)]
    #for traj in pose_list:
    #    ctl_interface.clear_pose_list()
    #    ctl_interface.add_pose_list(traj)
    #    ctl_interface.plan_and_execute_cartesian()








