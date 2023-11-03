#!/usr/bin/env python3

import rospy
import os
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseWithCovarianceStamped as Pose
from nav_msgs.msg import Odometry
from math import cos, sin, atan2, sqrt
from matplotlib import pyplot as plt

from EKF import ExtendedKalmanFilter

class Fusion:
    def __init__(self):
        rospy.Subscriber('/gps', Pose, self.gpsCallback)
        rospy.Subscriber('/radar_odometry', Odometry, self.odometryCallback)
        rospy.Subscriber('/gt_odom', Odometry, self.gtCallback)
        rospy.on_shutdown(self.shutdown)
        self.posePub = rospy.Publisher('/pred', Odometry, queue_size = 10)
        self.EKF = ExtendedKalmanFilter()
        
        self.gt_list = [[], []]
        self.est_list = [[], []]
        self.initial = False

        self.prev_pose = np.zeros(3)
        self.prev_gps = np.zeros(3)

    def shutdown(self):
        print("shuting down fusion.py")

    def predictPublish(self):
        
        predPose = Odometry()
        predPose.header.frame_id = 'origin'
        # change to the state x and state y from EKF
        predPose.pose.pose.position.x = self.EKF.pose[0]
        predPose.pose.pose.position.y = self.EKF.pose[1]
        
        # Change to the state yaw from EKF
        quaternion = quaternion_from_euler(0, 0, self.EKF.pose[2])
        # print(self.EKF.pose[0], self.EKF.pose[1], self.EKF.pose[2])
        predPose.pose.pose.orientation.x = quaternion[0]
        predPose.pose.pose.orientation.y = quaternion[1]
        predPose.pose.pose.orientation.z = quaternion[2]
        predPose.pose.pose.orientation.w = quaternion[3]
        
        # Change to the covariance matrix of [x, y, yaw] from EKF
        predPose.pose.covariance = tuple(self.EKF.S.ravel().tolist())
                                    
        self.posePub.publish(predPose)
    
    def odometryCallback(self, data):
        odom_x = data.pose.pose.position.x
        odom_y = data.pose.pose.position.y
        odom_quaternion = [
            data.pose.pose.orientation.x, data.pose.pose.orientation.y,
            data.pose.pose.orientation.z, data.pose.pose.orientation.w
        ]
        _, _, odom_yaw = euler_from_quaternion(odom_quaternion)
        odom_covariance = np.array(data.pose.covariance).reshape(6, 6)
        
        # Design the control of EKF state from radar odometry data
        # The data is in global frame, you may need to find a way to convert it into local frame
        # Ex. 
        #     Find differnence between 2 odometry data 
        #         -> diff_x = ???
        #         -> diff_y = ???
        #         -> diff_yaw = ???
        #     Calculate transformation matrix between 2 odometry data
        #         -> transformation = last_odom_pose^-1 * current_odom_pose
        #     etc.
        diff_x = odom_x - self.prev_pose[0]
        diff_y = odom_y - self.prev_pose[1]
        diff_yaw =  odom_yaw - self.prev_pose[2]
        # set the current pose to previous pose
        self.prev_pose[0] = odom_x
        self.prev_pose[1] = odom_y
        self.prev_pose[2] = odom_yaw

        control = np.zeros(3)
        control[0] = diff_x
        control[1] = diff_y
        control[2] = diff_yaw
        
        if not self.initial:
            self.initial = True
        else:
            # Update error covriance
            # self.EKF.R = [[odom_covariance[0,0],0,0,0,0,0],
            #               [0,odom_covariance[1,1],0,0,0,0],
            #               [0,0,odom_covariance[2,2],0,0,0],
            #               [0,0,0,0,0,0],
            #               [0,0,0,0,0,0],
            #               [0,0,0,0,0,0]]
            # self.EKF.R[0,0] = odom_covariance[0,0]
            # self.EKF.R[1,1] = odom_covariance[1,1]
            # self.EKF.R[2,2] = odom_covariance[2,2]
            self.EKF.predict(u = control)

        self.predictPublish()
        return
        
    def gpsCallback(self, data):
        gps_x = data.pose.pose.position.x
        gps_y = data.pose.pose.position.y
        gps_covariance = np.array(data.pose.covariance).reshape(6, 6)
        
        # Design the measurement of EKF state from GPS data
        # Ex. 
        #     Use GPS directly
        #     Find a approximate yaw
        #     etc.
        
        if not self.initial:
            # self.initial = ExtendedKalmanFilter(gps_x, gps_y)
            self.initial = True
            self.EKF.set_initial_pose(gps_x, gps_y, 0)
        else:
            # Update error covriance
            self.EKF.Q = [[gps_covariance[0,0],0,0],
                          [0,gps_covariance[1,1],0],
                          [0,0,gps_covariance[0,0]]]
            ang_approx = atan2(gps_y - self.prev_gps[1], gps_x - self.prev_gps[0])
            measurement = [gps_x, gps_y, ang_approx]
            self.EKF.update(z = measurement)
            
        self.prev_gps[0] = gps_x
        self.prev_gps[1] = gps_y
        self.predictPublish()
        return
    
    def gtCallback(self, data):
        self.gt_list[0].append(data.pose.pose.position.x)
        self.gt_list[1].append(data.pose.pose.position.y)
        if self.EKF is not None:
            # Change to the state x and state y from EKF
            self.est_list[0].append(self.EKF.pose[0])
            self.est_list[1].append(self.EKF.pose[1])
        return

    def plot_path(self):
        plt.figure(figsize=(10, 8))
        plt.xlabel('x')
        plt.ylabel('y')
        plt.grid(True)
        plt.plot(self.gt_list[0], self.gt_list[1], alpha=0.25, linewidth=8, label='Groundtruth path')
        plt.plot(self.est_list[0], self.est_list[1], alpha=0.5, linewidth=3, label='Estimation path')
        plt.title("KF fusion odometry result comparison")
        plt.legend()
        if not os.path.exists("/root/catkin_ws/result"):
            print("not exist")
            os.mkdir("/root/catkin_ws/result")
        plt.savefig("/root/catkin_ws/result/result.png")
        plt.show()
        return
    
if __name__ == '__main__':
    rospy.init_node('kf', anonymous=True)
    fusion = Fusion()
    rospy.spin()
    fusion.plot_path()