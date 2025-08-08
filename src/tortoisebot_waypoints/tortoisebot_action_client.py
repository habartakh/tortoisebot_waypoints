#! /usr/bin/env python3

import rospy
import actionlib
from tortoisebot_waypoints.msg import WaypointActionAction, WaypointActionGoal, WaypointActionResult, WaypointActionFeedback

def feedback_cb(msg):
 print ('Feedback received:', msg)

def call_server():

    client = actionlib.SimpleActionClient('tortoisebot_as', WaypointActionAction)

    client.wait_for_server()

    goal = WaypointActionGoal()
    goal.position.x = 0.0
    goal.position.y = 0.2
    goal.position.z = 0.0

    client.send_goal(goal, feedback_cb=feedback_cb)

    client.wait_for_result()

    result = client.get_result()

    return result

if __name__ == '__main__':

    try:
        rospy.init_node('action_client')
        result = call_server()
        print ('The result is:', result)
    except rospy.ROSInterruptException as e:
        print ('Something went wrong:', e)