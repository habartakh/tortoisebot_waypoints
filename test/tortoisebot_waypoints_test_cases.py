#! /usr/bin/env python3

from tortoisebot_waypoints.tortoisebot_action_server import WaypointActionClass
import actionlib
from tortoisebot_waypoints.msg import WaypointActionAction, WaypointActionGoal, WaypointActionResult, WaypointActionFeedback
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Point
from tf.transformations import euler_from_quaternion
import rospy
import rosunit
import unittest
import rostest
import time
import math

PKG = 'tortoisebot_waypoints'
NAME = 'waypoint_action_class_integration_test'


class TestWaypointActionClass(unittest.TestCase):

    def setUp(self):

        rospy.init_node('test_node')

        self.goal = Point()
        self.goal.x = rospy.get_param('~goal_x', 0.0)
        self.goal.y = rospy.get_param('~goal_y', 0.0)
        self.goal_yaw = rospy.get_param('~goal_yaw', 0.0)
        self.precision_xy = rospy.get_param('~precision_xy', 0.1)
        self.precision_yaw = rospy.get_param('~precision_yaw', 0.2)

        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.current_orientation = Quaternion()
        self.current_x = 0 
        self.current_y = 0
        
        self.final_yaw = 0
        self.final_x = 0
        self.final_y = 0

        # Call the action server directly from test script to avoid timing issues:
        # The timeout introduced previously (7s)in test functions is chosen randomly
        # And sometimes the tests start before the robot completes its trajectory
        self.client = actionlib.SimpleActionClient('tortoisebot_as', WaypointActionAction)
        self.client.wait_for_server()
        self.success = False


    def odom_callback(self, msg):

        self.current_orientation = msg.pose.pose.orientation
        self.current_x = msg.pose.pose.position.x 
        self.current_y = msg.pose.pose.position.y

    def euler_to_quaternion(self, msg):

        orientation_list = [msg.x, msg.y, msg.z, msg.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        return yaw

     # send goal to the server
    def action_server_call(self):

        action_goal = WaypointActionGoal()
        action_goal.position = self.goal
        self.client.send_goal(action_goal)


    def test_correct_position(self):
        
        self.action_server_call()
        # wait until the goal is reached
        self.success = self.client.wait_for_result(rospy.Duration(30.0))

        # Make sure that the goal is reached
        self.assertTrue(self.success)
        
        # Now we get the proper final values 
        self.final_x = self.current_x 
        self.final_y = self.current_y
        print("Final X:" , self.final_x)
        print("Final Y:" , self.final_y)
       

        x_diff = self.final_x - self.goal.x
        y_diff = self.final_y - self.goal.y
        print ("X difference : ", x_diff)
        print ("Y difference : ", y_diff)

        dist_to_goal = math.sqrt(pow(x_diff, 2) +
            pow(y_diff, 2))
        print ("dist_to_goal : ", dist_to_goal)

        self.assertTrue((dist_to_goal <= self.precision_xy), "Error: Position was not between the expected values.")

    def test_correct_rotation(self):
        
        print ("Current Orientation:",self.current_orientation )
        time.sleep(7)
        
        self.final_yaw = self.euler_to_quaternion(self.current_orientation)
        print("Final Yaw:", self.final_yaw )
        
        yaw_diff = self.final_yaw - self.goal_yaw
        print ("Yaw Diff:", yaw_diff)

        self.assertTrue((math.fabs(yaw_diff)<= self.precision_yaw), "Error: Rotation was not between the expected values.")

if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestWaypointActionClass)