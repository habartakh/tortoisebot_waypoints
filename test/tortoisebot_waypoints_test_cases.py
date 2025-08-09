#! /usr/bin/env python3

from tortoisebot_waypoints.tortoisebot_action_server import WaypointActionClass
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from tf.transformations import euler_from_quaternion
import rospy
import rosunit
import unittest
import rostest
import time

PKG = 'tortoisebot_waypoints'
NAME = 'waypoint_action_class_integration_test'

goal_x = 0.0
goal_y = 0.2
goal_yaw = 1.57 
precision_xy = 0.1
precision_yaw = 0.3

class TestWaypointActionClass(unittest.TestCase):

    def setUp(self):

        rospy.init_node('test_node')
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.current_orientation = Quaternion()
        self.current_x = 0 
        self.current_y = 0
        
        self.init_yaw = 0
        self.final_yaw = 0

        self.init_x = 0 
        self.final_x = 0
        self.init_y = 0 
        self.final_y = 0

    def odom_callback(self, msg):

        self.current_orientation = msg.pose.pose.orientation
        self.current_x = msg.pose.pose.position.x 
        self.current_y = msg.pose.pose.position.y

    def euler_to_quaternion(self, msg):

        orientation_list = [msg.x, msg.y, msg.z, msg.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        return yaw

    def test_correct_position(self):
        print ("Current position:")
        print(str(self.current_x) + " ; " + str(self.current_y))
        time.sleep(10)
        
        msg = rospy.wait_for_message("/odom", Odometry, timeout=2)

        self.final_x = msg.pose.pose.position.x 
        self.final_y = msg.pose.pose.position.y
        print("Final X:" , self.final_x)
        print("Final Y:" , self.final_y)
       
        x_diff = self.final_x - self.init_x
        y_diff = self.final_y - self.init_y
        print ("X difference : ", x_diff)
        print ("Y difference : ", y_diff)

        x_min = goal_x - precision_xy
        x_max = goal_x + precision_xy

        y_min = goal_y - precision_xy
        y_max = goal_y + precision_xy

        self.assertTrue((x_min <= x_diff <= x_max and y_min <= y_diff <= y_max ), "Integration error. Position was not between the expected values.")

    def test_correct_rotation(self):
        
        print ("Current Orientation:",self.current_orientation )
        time.sleep(10)
        
        #rospy.wait_for_message("/odom", Odometry, timeout=2)
        self.final_yaw = self.euler_to_quaternion(self.current_orientation)
        print("Final Yaw:", self.final_yaw )
        yaw_diff = self.final_yaw - self.init_yaw
        print ("Yaw Diff:", yaw_diff)

        yaw_min = goal_yaw - precision_yaw
        yaw_max = goal_yaw + precision_yaw


        self.assertTrue((yaw_min <= yaw_diff <= yaw_max), "Integration error. Rotation was not between the expected values.")

if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestWaypointActionClass)