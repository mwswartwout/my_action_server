#!/usr/bin/env python

import actionlib
import rospy
from my_action_server.msg import pose_pathAction, pose_pathFeedback, \
    pose_pathResult


class PosePathServer:
    feedback = pose_pathFeedback()
    result = pose_pathResult()

    def __init__(self):
        self.server = actionlib.SimpleActionServer('pose_path_action',
                                                   pose_pathAction,
                                                   self.callback,
                                                   False)
        self.server.start()

    def callback(self, goal):
        self.server.set_succeeded()


def main():
    rospy.init_node('pose_path_server')
    server = PosePathServer()
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
