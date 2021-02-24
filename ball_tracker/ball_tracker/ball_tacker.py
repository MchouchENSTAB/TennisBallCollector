#!/usr/bin/env python3

"""
This program is demonstration for face and object detection using haar-like features.
The program finds faces in a camera image or video stream and displays a red box around them.
Original C implementation by:  ?
Python implementation by: Roman Stanchak, James Bowman
Updated: Copyright (c) 2016, Tal Regev.
"""

import sys
import os
from optparse import OptionParser
from rclpy.qos import DurabilityPolicy, QoSProfile, HistoryPolicy, ReliabilityPolicy
from rclpy.node import Node
import rclpy
import sensor_msgs.msg

from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Pose, Twist, PoseArray




class Tracker:
    """
    A basic color tracker, it will look for colors in a range and
    create an x and y offset valuefrom the midpoint
    """

    def __init__(self, height, width, color_lower, color_upper):
        self.color_lower = color_lower
        self.color_upper = color_upper
        self.midx = int(width / 2)
        self.midy = int(height / 2)
        self.xoffset = 0
        self.yoffset = 0

    def draw_arrows(self, frame):
        """Show the direction vector output in the cv2 window"""
        #cv2.putText(frame,"Color:", (0, 35), cv2.FONT_HERSHEY_SIMPLEX, 1, 255, thickness=2)
        cv2.arrowedLine(frame, (self.midx, self.midy),
                        (self.midx + self.xoffset, self.midy - self.yoffset),
                        (0, 0, 255), 5)
        return frame

    def track(self, frame):
        """Simple HSV color space tracking"""
        # resize the frame, blur it, and convert it to the HSV
        # color space
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # construct a mask for the color then perform
        # a series of dilations and erosions to remove any small
        # blobs left in the mask
        mask = cv2.inRange(hsv, self.color_lower, self.color_upper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # find contours in the mask and initialize the current
        # (x, y) center of the ball
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0]
        center = None

        count = 0
        centers = []
        for c in cnts:
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            
            
            # only proceed if the radius meets a minimum size
            if radius < 50:
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                cv2.circle(frame, (int(x), int(y)), int(radius),
                           (0, 255, 255), 2)
                cv2.circle(frame, center, 1, (0, 0, 255), -1)
                centers.append(list(center))
                self.xoffset = int(center[0] - self.midx)
                self.yoffset = int(self.midy - center[1])
                count += 1 
            
            else:
                self.xoffset = 0
                self.yoffset = 0
                
        else:
            self.xoffset = 0
            self.yoffset = 0
        # print("nb detected balls : ", count)
        return centers, frame

class TrackerPub(Node):
    def __init__(self):
        super().__init__('trackerpub')
        
        qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_ALL, depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )


        self.image_subscription = self.create_subscription(sensor_msgs.msg.Image, '/zenith_camera/image_raw', self.image_callback, qos_profile)

        self.balls_publisher = self.create_publisher(PoseArray, 'balls_pose', qos_profile)
        self.robot_publisher = self.create_publisher(Pose, 'robot_pose', qos_profile)
    
    
    def distance(self, x, y, x0, y0):
        return np.sqrt((x - x0)**2 + (y - y0)**2)

    def image_callback(self, imgmsg):
        br = CvBridge()
        ball_poses = PoseArray()
        robot_pose = Pose()
        ball_poses.header._frame_id ='map'
        img = br.imgmsg_to_cv2(imgmsg, "bgr8")
        ball_centers, robot_center, frame = self.computerVision(img)
        
        
        # print("Ball center : ", ball_centers)
        # print("Robot center : ", robot_center)

        
        base1 = (7, 13)
        base2 = (-7, -13)

        robot_pose.position.x = robot_center[0][0]
        robot_pose.position.y = robot_center[0][1]



        for ball in ball_centers:
            ball_pose = Pose()
            dis1 = self.distance(ball[0], ball[1], base1[0], base1[1])
            dis2 = self.distance(ball[0], ball[1], base2[0], base2[1])
            
            if dis1 < dis2:
                vec = (base1[0] - ball[0], base1[1] - ball[1])
                yaw = np.arctan2(vec[1], vec[0])

            else:
                vec = (base2[0] - ball[0], base2[1] - ball[1])
                yaw = np.arctan2(vec[1], vec[0])
            
            print(yaw)
            q = self.quaternion_from_euler(0, 0, yaw)
            ball_pose.position.x = ball[0]
            ball_pose.position.y = ball[1]
            ball_pose.orientation.x = q[0]
            ball_pose.orientation.y = q[1]
            ball_pose.orientation.z = q[2]
            ball_pose.orientation.w = q[3]
            ball_poses.poses.append(ball_pose)

        
        self.balls_publisher.publish(ball_poses)
        self.robot_publisher.publish(robot_pose)
        cv2.imshow("Frame", frame)
        cv2.waitKey(6)
        
        # cv2.imwrite('/tmp/post_im.png', img)

    def quaternion_from_euler(self, roll, pitch, yaw):
        """
        Converts euler roll, pitch, yaw to quaternion (w in last place)
        quat = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """
        import math
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = [0] * 4
        q[0] = cy * cp * cr + sy * sp * sr
        q[1] = cy * cp * sr - sy * sp * cr
        q[2] = sy * cp * sr + cy * sp * cr
        q[3] = sy * cp * cr - cy * sp * sr

        return q

    def getBallPosition(self, centers):
        tr_matrix = np.array([[400, -688]]).T
        rot_matrix = np.array([[0, -1], [1, 0]])
        courtWidthPix = 800
        courtWidthM = 16
        unit = courtWidthM / courtWidthPix
        for center in centers:
            center_matrix = np.array([[center[0], center[1]]]).T
            center_matrix = rot_matrix @ center_matrix + tr_matrix
            center_matrix[1, 0] = -center_matrix[1, 0]
            center[0] = center_matrix[0, 0] * unit
            center[1] = center_matrix[1, 0] * unit
        return centers

    def getProjectedFrame(self, frame):

        w, h = 1376, 800
        # y vers le bas & x vers la droite
        bg = [32,682]
        hg = [32, 37]
        hd = [1247,37]
        bd = [1247, 682]

        pts1 = np.float32([hg, hd, bg, bd])
        pts2 = np.float32([[0,0],[w,0],[0,h],[w, h]])

        M = cv2.getPerspectiveTransform(pts1,pts2)

        dst = cv2.warpPerspective(frame,M,(w,h))
        return dst

    def computerVision(self, frame):
        frame = self.getProjectedFrame(frame)

        green_lower = (30, 50, 50)
        green_upper = (70, 255, 255)
        red_lower = (0, 50, 50)
        red_upper = (10, 255, 255)
        
        height, width = frame.shape[0], frame.shape[1]

        greentracker = Tracker(height, width, green_lower, green_upper)
        redtracker = Tracker(height, width, red_lower, red_upper)
        
        ball_centers, frame = greentracker.track(frame)
        robot_centers, frame = redtracker.track(frame)
        
        ball_centers = self.getBallPosition(ball_centers)
        robot_center = self.getBallPosition(robot_centers)

        return ball_centers, robot_center, frame 




def main(args=None):
    rclpy.init(args=args)
    trackerpub = TrackerPub()
    rclpy.spin(trackerpub)
    trackerpub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()