import rospy
import numpy as np

import roboticstoolbox as rtb
import spatialmath as sm


from numpy import pi, typing

from spatialmath import SE3

# messages
from std_msgs.msg import String
from geometry_msgs.msg import Point,Pose
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from robotics_lab5.msg import PincherPose

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
        self.positionSub = rospy.Subscriber("/target_pincher_pose", PincherPose,self.ikinematics)
    
        self.jStateSub = rospy.Subscriber("/dynamixel_workbench/joint_states", JointState,self.fkinematics)
            
        self.jTrajecPub = rospy.Publisher('/joint_trajectory', JointTrajectory, queue_size=0)
        
    def rosCommunication(self):

        while not rospy.is_shutdown():
            #print(state)
            rospy.loginfo("pincher running") 
        
            rospy.sleep(1)

    def fkinematics(self,JointState_msg):
        q = JointState_msg.position
        
        MTH = self.robot.fkine(q)
        position  =MTH.t

        pass

    def ikinematics(self,PincherPose_msg):
        L1 = self.L1
        L2 = self.L2
        L3 = self.L3
        L4 = self.L4
        
        x = Point_msg.x
        y = Point_msg.y
        z = Point_msg.z
        
        theta = PincherPose_msg.theta
        Point_msg = PincherPose_msg.Point 
        
        q1 = np.arctan2(y,x)
        
        q4 = theta
        
        wrist_center = SE3.Trans(-L4, 0, 0)*SE3.Ry(-q4)*SE3.Rz(q1)*SE3.Trans(x, y, z)
        pos_wrist_center =wrist_center.t
        
        x_c = pos_wrist_center[0]
        y_c = pos_wrist_center[1]
        z_c = pos_wrist_center[2]
        
        L = np.sqrt((z_c-L1)**2 + x_c**2 +y_c**2)

        c_q3 = (L-(L2**2 + L3**2) )/(2*L2*L3)
        s_q3 = np.sqrt(1 - c_q3**2)
        q3 = np.arctan2(s_q3,c_q3)
        
        alpha = np.arctan2(L3*c_q3 , L2 + L3*s_q3)
        gamma = np.arctan2(z-L1, np.sqrt(x**2+y**2) )
        
        q2 = gamma - alpha

        q = [q1,q2,q3,q4]
        
        print(q)

        pass



if __name__ == '__main__':
    
    rospy.init_node('pincher_node',anonymous=False)
    try:
        
        robot = pincher()
        robot.rosCommunication()

    except rospy.ROSInterruptException:
        pass