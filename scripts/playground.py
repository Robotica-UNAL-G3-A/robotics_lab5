L1=152.05
import rospy
import numpy as np
import matplotlib.pyplot as plt
import time

import roboticstoolbox as rtb
import spatialmath as sm


from numpy import pi

from spatialmath import SE3
from spatialmath.base import tr2rpy


L2=136.82
L3=74.12
L4=108.4
Link1 = rtb.robot.DHLink(qlim=[-np.pi/2,np.pi/2],d=L1,a=0,alpha=np.pi/2)
Link2 = rtb.robot.DHLink(qlim=[-np.pi/2, np.pi/2],d=0,a=L2,alpha=0,offset=np.pi/2)
Link3 = rtb.robot.DHLink(qlim=[-np.pi/2, np.pi/2],d=0,a=L3,alpha=0)
Link4 = rtb.robot.DHLink(qlim=[-np.pi/2, np.pi/2],d=0,a=L4,alpha=0)
robot = rtb.robot.DHRobot([Link1, Link2, Link3, Link4],name='Pincher rtb')


def main():
    q_in = np.radians([-84.96,  -50.68, -144.43,  101.95])
    pose = fkinematics(robot,q_in)
    
    q_out =ikinematics(pose)
    pose2 = fkinematics(robot,q_out)
    
    pose_ini = np.radians([ 16.72, -189.63,  139.56,   0.258])
    pose_ini.shape = (4,1)
    pose_fin = np.radians([ 16.72, -189.63,  139.56,   0.258])
    pose_fin.shape = (4,1)
    
    t =np.linspace(0,1,10)[np.newaxis]
    
    #pose_inter =pose_ini+ np.dot(t,pose_fin-pose_ini)/100
    print(pose_ini)
    
    
    #q3 =ikinematics(pose3)
    
    
    print("qs")
    print(q_in)
    
    print(q_out)
    #print(q3)
    print("pose")
    print(pose)
    print(pose2)
    
    #robot.plot(q_in)
    #input()
    
   # plt.figure(1)
    #robot.plot(q_out)
    #input()
        


def fkinematics(robot,q):
        #rostopic pub -r 10 /dynamixel_workbench/joint_states trajectory_msgs/JointTrajectory  '{linear:  {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'
        MTH = robot.fkine(q)
        
        
        pos_vector  = MTH.t
        
        rpy_angles= MTH.rpy()
        print("rpy" +str(rpy_angles))
        
        print("MTH" +str(MTH))
        theta = rpy_angles[1]
        pose = np.append(pos_vector,theta)
        return pose
    
def ikinematics(pose):
        L1=152.05
        L2=136.82
        L3=74.12
        L4=108.4
        
        x = pose[0]
        y = pose[1]
        z = pose[2]
        theta = pose[3]
        
        q1 = np.arctan2(y,x)
        
        MTH6 = SE3.Trans(x, y, z)*SE3.RPY(pi/2,theta,q1)
        print("MTH6")
        print(MTH6)

        wrist_center = MTH6*SE3.Trans(-L4, 0, 0)
        pos_wrist_center =wrist_center.t

        x_c = pos_wrist_center[0]
        y_c = pos_wrist_center[1]
        z_c = pos_wrist_center[2]

        L = np.sqrt((z_c-L1)**2 + x_c**2 +y_c**2)
        
        # elbow up
        
        c_q3 = (L**2-(L2**2 + L3**2) )/(2*L2*L3)
        s_q3 = -np.sqrt(1 - c_q3**2)
        q3 = np.arctan2(s_q3,c_q3)
        
        alpha = np.arctan2(L3*c_q3 , L2 + L3*s_q3)
        gamma = np.arctan2(np.sqrt(x_c**2+y_c**2), z_c-L1  )
        
        q2 =  -gamma - alpha  #-pi/2
        
        
        q4 = q2-q3-theta

        #q_offset =np.array([0,pi/2,0,0])
        q = np.array([q1,q2,q3,q4])
        return q
if __name__ == '__main__':
    main()
 