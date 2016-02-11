#!/usr/bin/env python

import actionlib
import rospy
from my_action_server.msg import pose_pathAction, pose_pathGoal


def main():
    rospy.init_node('pose_path_client')
    client = actionlib.SimpleActionClient('pose_path', pose_pathAction)
    rospy.loginfo("Waiting for pose_path action server")
    client.wait_for_server()
    rospy.loginfo("Connected to pose_path action server!")

    goal = pose_pathGoal()
    client.send_goal(goal)
    client.wait_for_result()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
