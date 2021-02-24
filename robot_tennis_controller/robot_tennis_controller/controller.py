import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, Twist, PoseArray
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, HistoryPolicy, ReliabilityPolicy
import numpy as np

import math
 

def quaternion_to_euler(q):
        (x, y, z, w) = (q[0], q[1], q[2], q[3])
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians


def sawtooth(x):
    return (x + np.pi) % (2*np.pi) - np.pi


def distance(x, y, x0, y0):
            return np.sqrt((x - x0)**2 + (y - y0)**2)

class Controller(Node):

    def __init__(self):
        super().__init__('controller')

        timer_period = 0.05
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Publisher
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 25)
        qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_ALL, depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        # Subscriber
        self.robot_subscription = self.create_subscription(Pose, 'robot_pose', self.robot_position_callback, 25)
        self.robot_angle_subscription = self.create_subscription(Imu, 'imu_plugin/out', self.robot_angle_callback, 25)
        self.ball_subscription = self.create_subscription(PoseArray, 'balls_pose', self.ball_position_callback, qos_profile)

        self.robot_odometry_subscription = self.create_subscription(Odometry, 'odom', self.odometry_callback, 25)

        # Changing control law
        self.isNear = False
        self.distance_near = 3
        self.distance_threshold = 0.8

        # Ball and robot position
        self.X = np.zeros((3, 1))
        self.B = np.zeros((3, 1))
        self.A = np.zeros((3, 1))
        self.A[0, 0] = - self.distance_near

    def robot_position_callback(self, data):
        self.X[0, 0] = data.position.x  
        self.X[1, 0] = data.position.y

    def odometry_callback(self, data):
        self.X[0, 0] = data.pose.pose.position.x
        self.X[1, 0] = data.pose.pose.position.y
        (roll, pitch, yaw) = quaternion_to_euler ([data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w])
        self.X[2, 0] = yaw

    def robot_angle_callback(self, data):
        (roll, pitch, yaw) = quaternion_to_euler ([data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w])
        self.X[2, 0] = yaw
    
    def ball_position_callback(self, data):
        data = data.poses[0]
        print(data)

        (roll, pitch, yaw) = quaternion_to_euler([data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w])
        self.B = np.array([[data.position.x], [data.position.y], [yaw]])
        R = np.array([[np.cos(yaw), np.sin(yaw), 0], [-np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
        self.A = self.B + R @ np.array([[- self.distance_near], [0], [0]])

    def timer_callback(self):

        # Position error processing in different cases
        if self.isNear:
            e = (self.B - self.X)
            if np.linalg.norm(e[:2]) > self.distance_near + 2*self.distance_threshold:
                self.isNear = False
        else:
            e = (self.A - self.X)
            if np.linalg.norm(e) < self.distance_threshold:
                self.isNear = True        

        # Computing speed and angle
        if self.isNear:
            if np.linalg.norm(e[:2]) < self.distance_threshold:
                K_speed = 0.0
                K_rotation = 0.5
            else:
                K_speed = 0.5
                K_rotation = 0.5

            speed = K_speed * np.linalg.norm(e[:2])
            theta = K_rotation * sawtooth(self.B[2, 0] - self.X[2, 0])
        else :
            K_speed = 0.4
            K_rotation = 1.5
            speed = K_speed * 3.0
            theta = K_rotation * sawtooth(np.arctan2(e[1, 0], e[0, 0]) - self.X[2, 0])

        
        
        # Building message
        msg = Twist()
        msg.linear.x = speed
        msg.angular.z = theta

        # Publishing
        self.publisher.publish(msg)
        self.get_logger().info(str(distance(self.B[0, 0], self.B[1, 0], self.X[0, 0], self.X[1, 0])))
        #self.get_logger().info("Ball_Position: {}, {}, {}".format(self.B[0, 0],  self.B[1, 0], self.B[2, 0]))
        #self.get_logger().info("Robot_Position: {}, {}".format(self.X[0, 0], self.X[1, 0]))
        #self.get_logger().info("Error: {}, {}, {}".format(self.isNear, speed, theta))
        #self.get_logger().info("Publishing: {}, {}, {}".format(msg.linear.x, msg.linear.y, msg.angular.z))


def main(args=None):
    rclpy.init(args=args)
    controller = Controller()
    rclpy.spin(controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()