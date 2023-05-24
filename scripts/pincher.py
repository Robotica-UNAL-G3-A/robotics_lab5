import rospy
import numpy as np

from numpy import pi

# messages
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class pincher():
    def __init__(self):
        # Robot parameters [mm]
        self.L1=152.05
        self.L2=136.82
        self.L3=74.12
        self.L4=108.4
        
        # ROS communications
        rospy.loginfo("pincher start") 

        self.jStateSub = rospy.Subscriber("/dynamixel_workbench/joint_states", JointState,self.fkinematics)
    
        while not rospy.is_shutdown():
            #print(state)
            rospy.loginfo("pincher running") 
        
            rospy.sleep(1)

    def fkinematics(self,msg):
        pass

    def ikinematics(self,q):
        pass



if __name__ == '__main__':
    
    rospy.init_node('pincher_node',anonymous=False)
    try:
        
        robot = pincher()

    except rospy.ROSInterruptException:
        pass