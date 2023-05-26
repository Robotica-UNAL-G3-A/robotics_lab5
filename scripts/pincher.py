#!/usr/bin/env python3

import rospy
import numpy as np
import time

import roboticstoolbox as rtb
import spatialmath as sm


from numpy import pi, typing

from spatialmath import SE3
from spatialmath.base import tr2rpy

# messages
from std_msgs.msg import String
from geometry_msgs.msg import Point,Pose
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from robotics_lab5.msg import PincherPose


class Pincher():
    def __init__(self):
        # Robot parameters [mm]
        self.L1=152.05
        self.L2=136.82
        self.L3=74.12
        self.L4=108.4
        Link1 = rtb.robot.DHLink(qlim=[0,np.pi],d=self.L1,a=0,alpha=np.pi/2)
        Link2 = rtb.robot.DHLink(qlim=[0, np.pi/2],d=0,a=self.L2,alpha=0,offset=np.pi/2)
        Link3 = rtb.robot.DHLink(qlim=[0, np.pi],d=0,a=self.L3,alpha=0)
        Link4 = rtb.robot.DHLink(qlim=[0, np.pi],d=0,a=self.L4,alpha=0)
        self.robot = rtb.robot.DHRobot([Link1, Link2, Link3, Link4],name='Pincher rtb')

        
        # ROS communications
        rospy.loginfo("pincher start") 
        self.positionSub = rospy.Subscriber("/target_pincher_pose", PincherPose,self.ikinematics)
    
        self.jStateSub = rospy.Subscriber("/dynamixel_workbench/joint_states", JointState,self.fkinematics)
            
        self.jTrajecPub = rospy.Publisher('/joint_trajectory', JointTrajectory, queue_size=10)
        
        self.positionPub = rospy.Publisher("/real_pincher_pose", PincherPose,queue_size=10)
        
    def rosCommunication(self):
        # calibration data
        caliTraj = [[0, 0, 0, 0, 0 ],
                    [-90, 0, 0, 0, 0 ],                 
                    [90, 0, 0, 0, 0 ],
                    [0, 90, 0, 0, 0 ],
                    [0, 0, 0, 0, 0 ],
                    [0, -90, 0, 0, 0 ],
                    [0, 0, 90, 0, 0 ],
                    [0, 0, -90, 0, 0 ],
                    [0, 0, 0, 90, 0 ],
                    [0, 0, 0, -90, 0 ],
                    [0, 0, 0, 0, -90 ],
                    [0, 0, 0, 0, 0 ]
                    ]
        
        q5_close =-100
        # min radius
        r_min1 = [-84.96,  -50.68, -144.43,  101.95, q5_close ]
        r_min2 = [100, -50.68, -144.43,  101.95, q5_close ]
        rminTraj = [r_min1, r_min2]

        # Max radius
        r_max1 = [-59.47,  -89.65,  -20.51,   19.92, q5_close]

        r_max2 = [77.47,  -89.65,  -20.51,   19.92, q5_close]
        rmaxTraj = [r_max2,r_max1]
        
        # Gripper close q5=56 

        #triangle
        t1 = [-17.29,  -45.12, -123.34,   70.31, q5_close]
        t2 = [-21.09,  -57.4, -101.37,   70.31, q5_close]
        t3 = [ -31.35,  -56.54, -108.11,   71.19, q5_close]
        t4 = [-17.29,  -45.12, -123.34,   70.31, q5_close]
        

        triangTraject = [t1,t2,t3,t4]

        # cuadrado
        
        c1 = [-45.41,  -52.73, -120.70,   82.32, q5_close]
        c2 = [-45.12,  -60.64, -103.42,   79.69, q5_close]
        c3 = [-53.32,  -62.11, -101.07,   79.69, q5_close]
        c4 = [-53.91,  -55.96, -115.43,   79.98, q5_close]

        squareTraject = [c1,c2,c3,c4]
        
        currentTraject = rminTraj + rmaxTraj+ triangTraject + squareTraject
        
        #currentTraject =caliTraj
        
        while not rospy.is_shutdown():
            #print(state)
            #if ():
            #    rospy.loginfo("pincher running") 
            rospy.sleep(5)
            
            for idx,q in enumerate(currentTraject):
                self.publishTrajectory(q)
                rospy.loginfo("pos " + str(idx) + " " +str(q))
                rospy.sleep(10)            
                    
            rospy.sleep(5)

    def fkinematics(self,JointState_msg):
        #rostopic pub -r 10 /dynamixel_workbench/joint_states trajectory_msgs/JointTrajectory  '{linear:  {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'
        q_extended = JointState_msg.position
        q = q_extended[0:-1]  
        MTH = self.robot.fkine(q)
        
        real_pose = PincherPose()
        
        pos_vector  = MTH.t
        point = Point()
        point.x = pos_vector[0]
        point.y = pos_vector[1]
        point.z = pos_vector[2]
        real_pose.point = point
        
        rpy_angles= MTH.rpy()
        real_pose.theta = rpy_angles[1]
        #rospy.loginfo(real_pose)
        #rospy.loginfo(real_pose.point)
        #vector_pos = [real_pose.point.x, real_pose.point.y, real_pose.point.z, real_pose.theta]
        
        self.positionPub.publish(real_pose)
        #rospy.loginfo("pincher fkinematics pose"+ str(np.round(vector_pos,3))) 
    
    def ikinematics(self,PincherPose_msg):
        #rostopic pub -r 10 /target_pincher_pose robotics_lab5/PincherPose  '{point:  {x: 100, y: 200.0, z: 20.0}, theta: -0.5}'
        L1 = self.L1
        L2 = self.L2
        L3 = self.L3
        L4 = self.L4
        
        Point_msg = PincherPose_msg.point 
        x = Point_msg.x
        y = Point_msg.y
        z = Point_msg.z
        
        theta = PincherPose_msg.theta
        
        q1 = np.arctan2(y,x)
        
        q4 = theta
        MTH6 = SE3.Trans(x, y, z)*SE3.Rz(q1)*SE3.Ry(q4)

        wrist_center = MTH6*SE3.Trans(-L4, 0, 0)
        pos_wrist_center =wrist_center.t

        x_c = pos_wrist_center[0]
        y_c = pos_wrist_center[1]
        z_c = pos_wrist_center[2]

        L = np.sqrt((z_c-L1)**2 + x_c**2 +y_c**2)

        c_q3 = (L**2-(L2**2 + L3**2) )/(2*L2*L3)
        s_q3 = np.sqrt(1 - c_q3**2)
        q3 = np.arctan2(s_q3,c_q3)
        
        alpha = np.arctan2(L3*c_q3 , L2 + L3*s_q3)
        gamma = np.arctan2(z-L1, np.sqrt(x**2+y**2) )
        
        q2 = gamma - alpha

        q = [q1,q2,q3,q4]
        gripperIsOpen = True
        
        if gripperIsOpen:
            q5 = 0
        else :
            q5 = 100    
        q.append(q5)
        
        self.publishTrajectory(q)
        
    def publishTrajectory(self,q):    
        # q input in deg
        
        state = JointTrajectory()        
        state.header.stamp = rospy.Time.now()
        state.joint_names = ["joint_1","joint_2","joint_3","joint_4","joint_5"]
        point = JointTrajectoryPoint()
        point.positions = list(np.multiply(pi/180.0,q))  # conversion
        point.time_from_start = rospy.Duration(0.5)
        state.points.append(point)
        self.jTrajecPub.publish(state)
        
        rospy.loginfo("pincher ikinematics "+ str(np.round(q,3)) )
        
        rospy.loginfo("pincher state "+ str(state))
        
        pass



if __name__ == '__main__':
    
    rospy.init_node('pincher_node',anonymous=False)
    try:
        
        robot = Pincher()
        
            
        robot.rosCommunication()

    except rospy.ROSInterruptException:
        pass