#!/usr/bin/env python

import actionlib
import rospy
from geometry_msgs.msg import Pose, PoseStamped
from my_action_server.msg import pose_pathAction, pose_pathGoal
from nav_msgs.msg import Path
from std_msgs.msg import Header


# Forward = (1,0)
# Left = (0,1)
# Back = (-1,0)
# Right = (0,-1)


def generate_square_path():
    path = Path()

    pose = generate_pose(1, 0)
    path.poses.append(pose)

    pose = generate_pose(0, 1)
    path.poses.append(pose)
    path.poses.append(pose)
    path.poses.append(pose)


# Takes a Pose and adds a header to make it a PoseStamped
def stamp_pose(pose):
    pose_stamped = PoseStamped()
    pose_stamped.pose = pose

    header = Header()
    header.stamp = rospy.Time.now()
    pose_stamped.header = header

    return pose_stamped


def generate_pose(x=0, y=0, quaternion=None):
    if quaternion is None:
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
