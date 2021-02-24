import numpy as np
import math

import rclpy

from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Bool
from std_msgs.msg import Int16
from rclpy.node import Node
from rclpy.qos import QoSProfile






class TennisCollectorNavigationSimple(Node):

    def __init__(self):
        
        super().__init__('navigation')
        
        self.is_ball_collected = False
        self.is_ball_on_court = True
        self.balls_pose = []
        self.pose_ball_x = 0.0
        self.pose_ball_y = 0.0
        self.pose_rob_x = 0.0
        self.pose_rob_y = 0.0

        self.state = 0
        self.x_wp = 0.0
        self.y_wp = 0.0
        self.move = False

        qos = QoSProfile(depth=10)

        timer_period = 0.05
        self.timer = self.create_timer(timer_period, self.navigation)

        # PUBLISHER INITIALIZATION
        #-------------------------------------------------------------------
        self.wp_pub = self.create_publisher(Pose, 'waypoint', qos)

        self.move_pub = self.create_publisher(Bool, 'move', qos)

        self.state_pub = self.create_publisher(Int16, 'state', qos)
        #-------------------------------------------------------------------


        # PUBLISHER SUBSCIPTER INITIALIZATION
        #-------------------------------------------------------------------
        self.ball_sub = self.create_subscription(
            PoseArray,
            'balls_pose',
            self.ball_callback,
            qos)
        
        self.rob_sub = self.create_subscription(
            Pose,
            'robot_pose',
            self.rob_callback,
            qos)
        """
        self.is_ball_collected_sub = self.create_subscription(
            Bool,
            'ibc',
            self.ibc_callback,
            qos)

        self.is_ball_on_court_sub = self.create_subscription(
            Bool,
            'iboc',
            self.iboc_callback,
            qos)
        #-------------------------------------------------------------------
        """

    def ball_callback(self, msg):
        
        self.balls_pose = msg.poses
        self.pose_ball_x = self.balls_pose[0].position.x
        self.pose_ball_y = self.balls_pose[0].position.y

    def rob_callback(self, msg):

        self.pose_rob_x = msg.pose.position.x
        self.pose_rob_y = msg.pose.position.y

    def ibc_callback(self, msg):

        self.is_ball_collected = msg.bool.data

    def iboc_callback(self, msg):

        self.is_ball_on_court = msg.bool.data


    def navigation(self):
        pose = Pose()
        bool = Bool()
        int = Int16()
        # STATE 1: GO TO THE OTHER SIDE OF THE COURT BYPASSING THE NET
        #-------------------------------------------------------------------
        if self.state == 1:

            if self.is_ball_on_court == True:
                if np.sign(self.pose_rob_y) != np.sign(self.pose_ball_y):
                    self.x_wp = 6*np.sign(self.poserob.x)
                    self.y_wp = 0.0
                else:
                    self.state = 2
            else :
                self.state = 3
        #-------------------------------------------------------------------


        # STATE 2: GO THE NEXT BALL TO COLLECT IT
        #-------------------------------------------------------------------
        if self.state == 2:

            if self.is_ball_collected == False:
                if np.sign(self.pose_rob_y) != np.sign(self.pose_ball_y):
                    self.state = 1
                else:
                    self.x_wp = self.pose_ball_x
                    self.y_wp = self.pose_ball_y
            else:
                self.state = 3
        #-------------------------------------------------------------------


        # STATE 3: BRING THE BALL INTO THE COLLECTING AREAS
        #-------------------------------------------------------------------
        if self.state == 3:

            if self.is_ball_collected == True:
                self.x_wp = 6.0*np.sign(self.pose_rob_y)*np.sign(abs(5-self.pose_rob_x))
                self.y_wp = 13.0*np.sign(self.pose_rob_y)*np.sign(abs(5-self.pose_rob_x))
            else:
                self.state = 2
        #-------------------------------------------------------------------


        # STATE 0: IDLE
        #-------------------------------------------------------------------
        else: # state == 0

            if self.is_ball_on_court == True:
                self.state = 2
                self.move == True
            else:
                if 6.0 < abs(self.pose_rob_x) < 7.0 and abs(self.pose_rob_y) < .5:
                    # stop robot
                    self.move = False
                else:
                    self.state = 1
                    self.move = True
        #-------------------------------------------------------------------


        # PUBLISHING WAYPONT (POSE) AND MOVING COMMAND (BOOL)
        #-------------------------------------------------------------------

        pose.position.x = self.x_wp
        pose.position.y = self.y_wp

        if self.state == 2:
            base = [6*np.sign(self.pose_ball_y), 13*np.sign(self.pose_ball_y)]
            vec = (base[0] - self.pose_ball_x, base[1] - self.pose_ball_y)
            yaw = np.arctan2(vec[1], vec[0])
            q = self.quaternion_from_euler(0, 0, yaw)
            pose.orientation.x = q[0]
            pose.orientation.y = q[1]
            pose.orientation.z = q[2]
            pose.orientation.w = q[3]
        if self.state == 3:
            yaw = -np.pi/4 + np.sign(self.pose_ball_y) * np.pi/2
            q = self.quaternion_from_euler(0, 0, yaw)
            pose.orientation.x = q[0]
            pose.orientation.y = q[1]
            pose.orientation.z = q[2]
            pose.orientation.w = q[3]
        else:
            yaw = np.sign(self.pose_rob_y) * np.pi/2
            q = self.quaternion_from_euler(0, 0, yaw)
            pose.orientation.x = q[0]
            pose.orientation.y = q[1]
            pose.orientation.z = q[2]
            pose.orientation.w = q[3]

        self.wp_pub.publish(pose)
        # self.get_logger().info("Publishing: {}, {}, {}".format(pose.x, pose.y, yaw))

        bool.data = self.move
        self.move_pub.publish(bool)
        int.data = self.state
        self.state_pub.publish(int)
        #-------------------------------------------------------------------
    def quaternion_from_euler(self, roll, pitch, yaw):
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        return [qx, qy, qz, qw]
    



def main(args=None):
    rclpy.init(args=args)
    navigation_node = TennisCollectorNavigationSimple()
    rclpy.spin(navigation_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    navigation_node.publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()