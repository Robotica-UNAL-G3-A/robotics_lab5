#!/usr/bin/env python3

import rospy
import numpy as np

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
        
    #def rosCommunication(self):

        while not rospy.is_shutdown():
            #print(state)
            rospy.loginfo("pincher running") 
        
            rospy.sleep(30)

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
        rospy.loginfo(real_pose)
        rospy.loginfo(real_pose.point)
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
        publishTrajectory(q)
        
    def publishTrajectory(self,q):    
        state = JointTrajectory()        
        state.header.stamp = rospy.Time.now()
        state.joint_names = ["joint_1","joint_2","joint_3","joint_4","joint_5"]
        point = JointTrajectoryPoint()
        point.positions = q
        point.time_from_start = rospy.Duration(0.5)
        state.points.append(point)
        self.jTrajecPub.publish(state)
        
        rospy.loginfo("pincher ikinematics "+ str(np.round(q,3)) )


if __name__ == '__main__':
    
    rospy.init_node('pincher_node',anonymous=False)
    try:
        
        robot = Pincher()
        #robot.rosCommunication()

    except rospy.ROSInterruptException:
        pass