#!/usr/bin/env python

import math

import actionlib
import rospy
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from my_action_server.msg import pose_pathAction, pose_pathGoal
from nav_msgs.msg import Path
from std_msgs.msg import Header, Bool

import utils


class ActionClient:
    def __init__(self):
        self.lidar_alarm = rospy.Subscriber('lidar_alarm',
                                            Bool,
                                            self.alarm_callback)

        self.client = actionlib.SimpleActionClient('pose_path_action',
                                                   pose_pathAction)

        rospy.loginfo("Waiting for pose_path action server")
        self.client.wait_for_server()
        rospy.loginfo("Connected to pose_path action server!")

    def alarm_callback(self, alarm):
        if alarm:
            path = generate_turn_path(math.pi / 8)
            goal = pose_pathGoal(path=path)
            self.client.send_goal(goal)
            self.client.wait_for_result()

    def do_square(self):
        path = generate_square_path()
        goal = pose_pathGoal(path=path)
        self.client.send_goal(goal)
        self.client.wait_for_result()


# Forward = (1,0)
# Left = (0,1)
# Back = (-1,0)
# Right = (0,-1)
def generate_square_path():
    path = Path()

    pose = generate_stamped_pose(0, 1)
    path.poses.append(pose)
    path.poses.append(pose)
    path.poses.append(pose)
    path.poses.append(pose)

    return path


def generate_turn_path(turn_increment):
    path = Path()

    quaternion = utils.convert_planar_phi_to_quaternion(turn_increment)
    pose = generate_stamped_pose(0, 0, quaternion)
    path.poses.append(pose)

    return path


# Takes a Pose and adds a header to make it a PoseStamped
def stamp_pose(pose):
    pose_stamped = PoseStamped()
    pose_stamped.pose = pose

    header = Header()
    header.stamp = rospy.Time.now()
    pose_stamped.header = header

    return pose_stamped


def generate_stamped_pose(x=0, y=0, quaternion=None):
    if quaternion is None:
        quaternion = Quaternion()
        quaternion.x = 0
        quaternion.y = 0
        quaternion.z = 0
        quaternion.w = 0

    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = 0
    pose.orientation = quaternion

    return stamp_pose(pose)


def main():
    rospy.init_node('pose_path_client')

    client = ActionClient()
    while not rospy.is_shutdown():
        client.do_square()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
