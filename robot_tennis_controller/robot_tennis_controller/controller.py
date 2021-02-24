import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, Twist, PoseArray
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, HistoryPolicy, ReliabilityPolicy
from std_msgs.msg import Int16, Bool
import numpy as np

import math
 

def euler_from_quaternion(q):
        (x, y, z, w) = (q[0], q[1], q[2], q[3])
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
        return [roll_x, pitch_y, yaw_z]

def sawtooth(x):
    return (x + np.pi) % (2*np.pi) - np.pi
    

class Controller(Node):

    def __init__(self):
        super().__init__('controller')

        timer_period = 0.05
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Publisher
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 25)

        # Subscriber
        self.robot_subscription = self.create_subscription(Pose, 'pose_rob', self.robot_position_callback, 25)
        self.robot_angle_subscription = self.create_subscription(Imu, 'imu_plugin/out', self.robot_angle_callback, 25)

        # FSM Subscription
        self.waypoint_subscription = self.create_subscription(Pose, 'waypoint', self.waypoint_callback, 25)
        self.state_subscription = self.create_subscription(Int16, 'state', self.state_callback, 25)
        self.move_subscription = self.create_subscription(Bool, 'move', self.move_callback, 25)

        self.state = 0
        self.move = False

        # self.robot_odometry_subscription = self.create_subscription(Odometry, 'odom', self.odometry_callback, 25)

        # Changing control law
        self.isNear = False
        self.distance_near = 3
        self.distance_threshold = 1.0

        # Waypoint and robot position
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
        (roll, pitch, yaw) = euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w])
        self.X[2, 0] = yaw

    def robot_angle_callback(self, data):
        (roll, pitch, yaw) = euler_from_quaternion([data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w])
        self.X[2, 0] = yaw

    def waypoint_callback(self, data):
        (roll, pitch, yaw) = euler_from_quaternion([data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w])
        self.B = np.array([[data.position.x], [data.position.y], [yaw]])
        R = np.array([[np.cos(yaw), np.sin(yaw), 0], [-np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
        self.A = self.B + R @ np.array([[- self.distance_near], [0], [0]])

    def move_callback(self, data):
        self.move = data.data

    def state_callback(self, data):
        self.state = data.data

    def timer_callback(self):

        if self.move:

            # Idle
            if self.state == 0:
                speed = 0.0
                theta = 0.0
            
            # Net bypass
            elif self.state == 1:

                # Position error processing in different cases
                if self.isNear:
                    e = (self.B - self.X)
                    if np.linalg.norm(e[:2]) > self.distance_near + 2*self.distance_threshold:
                        self.isNear = False
                else:
                    e = (self.A - self.X)
                    if np.linalg.norm(e) < self.distance_threshold:
                        self.isNear = True

                K_speed = 1.0
                K_rotation = 1.5
                speed = K_speed * 1.0
                theta = K_rotation * sawtooth(np.arctan2(e[1, 0], e[0, 0]) - self.X[2, 0])
                    
            
            # Goto next ball
            elif self.state == 2:

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
                        K_rotation = 0.0
                    else:
                        K_speed = 0.5
                        K_rotation = 0.5

                    speed = K_speed * np.linalg.norm(e[:2])
                    theta = K_rotation * sawtooth(self.B[2, 0] - self.X[2, 0])
                else :
                    K_speed = 1.0
                    K_rotation = 1.5
                    speed = K_speed * 1.0
                    theta = K_rotation * sawtooth(np.arctan2(e[1, 0], e[0, 0]) - self.X[2, 0])

            # Goto area
            elif self.state == 3:
                e = (self.B - self.X)
                K_speed = 2.0
                K_rotation = 1.0
                speed = K_speed * 1.0
                theta = K_rotation * sawtooth(np.arctan2(e[1, 0], e[0, 0]) - self.X[2, 0])

        else:
            speed = 0.0
            theta = 0.0

        # Building message
        msg = Twist()
        msg.linear.x = speed
        msg.angular.z = theta

        # Publishing
        self.publisher.publish(msg)
        self.get_logger().info("Robot: {}, {}, {}".format(self.X[0, 0], self.X[1, 0], self.X[2, 0]))
        self.get_logger().info("Error: {}, {}, {}".format(self.isNear, np.linalg.norm((self.A-self.X)[:2]), self.X[2, 0]-self.B[2, 0]))
        self.get_logger().info("Publishing: {}, {}, {}".format(msg.linear.x, msg.linear.y, msg.angular.z))


def main(args=None):
    rclpy.init(args=args)
    controller = Controller()
    rclpy.spin(controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()