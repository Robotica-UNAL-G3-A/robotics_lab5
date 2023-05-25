import rospy
import numpy as np

from numpy import pi


# messages
from std_msgs.msg import String
from geometry_msgs.msg import Point,Pose
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from robotics_lab5.msg import PincherPose
from robotics_lab5.scripts.pincher import Pincher



if __name__ == '__main__':
    
    rospy.init_node('pincher_node',anonymous=False)
    
    try:
        
        # min radius
        r_min1 = [65.04, 99.32, 5.57, 251.95 ]
        r_min2 = [252.83, 99.32, 5.57, 251.95 ]

        # Max radius
        r_max1 = [90.53, 60.35, 129.49, 169.92]

        r_max2 = [227.64, 60.35, 129.49, 169.92]

        # Gripper close q5=56 

        Home = [157.32, 157.32, 30.18, 171.39]

        #triangle
        t1 = [132.71, 104.88, 26.66,220.31]
        t2 = [128.91, 92.58, 48.63, 220.31]
        t3 = [118.65, 93.46,41.89, 221.19]
        t4 = [132.71, 104.88, 26.66,220.31]
        triangTraject = [t1,t2,t3,t4]

        # cuadrado
        c1 = [104.59,97.27, 29.30, 232.32]
        c2 = [104.88,89.36,46.58, 229.69]
        c3 = [96.68, 87.89, 48.93, 229.69 ]
        c4 = [ 96.09, 94.04, 34.57, 229.98]
        c5 = [104.59,97.27, 29.30, 232.32]
        squareTraject = [c1,c2,c3,c4]

        robot = Pincher()
        #robot.rosCommunication()

    except rospy.ROSInterruptException:
        pass
    
