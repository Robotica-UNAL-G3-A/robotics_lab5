import rospy
import numpy as np

import roboticstoolbox as rtb
import spatialmath as sm


from numpy import pi
from numpy import typing

# messages
from std_msgs.msg import String
from geometry_msgs.msg import Point,Pose
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class pincher():
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
        self.positionSub = rospy.Subscriber("/target", Point,self.ikinematics)
        
        self.jStateSub = rospy.Subscriber("/dynamixel_workbench/joint_states", JointState,self.fkinematics)

    def rosCommunication(self):

        while not rospy.is_shutdown():
            #print(state)
            rospy.loginfo("pincher running") 
        
            rospy.sleep(1)

    def fkinematics(self,JointState_msg):
        q = JointState_msg.position
        
        MTH = robot.fkine(q)

        
        pass

    def ikinematics(self,Point_msg):
        L1 = self.L1
        L2 = self.L2
        L3 = self.L3
        L4 = self.L4

        Point_msg.x
        Point_msg.y
        Point_msg.z

        pass



if __name__ == '__main__':
    
    rospy.init_node('pincher_node',anonymous=False)
    try:
        
        robot = pincher()
        robot.rosCommunication()

    except rospy.ROSInterruptException:
        pass