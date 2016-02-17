#!/usr/bin/env python

import math

import actionlib
import rospy
from geometry_msgs.msg import Pose, Twist
from my_action_server.msg import pose_pathAction, pose_pathFeedback, \
    pose_pathResult

import utils


class ActionServer:
    feedback = pose_pathFeedback()
    result = pose_pathResult()

    def __init__(self):
        rospy.loginfo("Initalizing ActionServer")
        self.move_speed = 1
        self.spin_speed = 1

        self.sample_dt = .01
        self.loop_timer = rospy.Rate(1 / self.sample_dt)

        self.current_pose = Pose()
        self.current_pose.position.x = 0
        self.current_pose.position.y = 0
        self.current_pose.position.z = 0
        self.current_pose.orientation.x = 0
        self.current_pose.orientation.y = 0
        self.current_pose.orientation.z = 0

        self.twist_cmd = Twist()
        self.twist_cmd.linear.x = 0
        self.twist_cmd.linear.y = 0
        self.twist_cmd.linear.z = 0
        self.twist_cmd.angular.x = 0
        self.twist_cmd.angular.y = 0
        self.twist_cmd.angular.z = 0

        self.twist_commander = rospy.Publisher('cmd_vel',
                                               Twist,
                                               queue_size=1)
        rospy.loginfo("twist_commander on topic cmd_vel created")

        self.server = actionlib.SimpleActionServer('pose_path_action',
                                                   pose_pathAction,
                                                   self.callback,
                                                   False)
        rospy.loginfo("Starting server on pose_path_action")
        self.server.start()
        rospy.loginfo("Started server successfully")

    def get_yaw_and_dist(self, goal_pose):
        x = goal_pose.position.x - self.current_pose.position.x
        y = goal_pose.position.y - self.current_pose.position.y
        yaw = math.atan2(y, x)
        dist = math.sqrt(x + y)
        return [yaw, dist]

    def do_spin(self, spin_angle):
        timer = 0
        final_time = math.fabs(spin_angle) / self.spin_speed
        self.twist_cmd.angular.z = utils.sgn(spin_angle) * self.spin_speed

        while timer < final_time:
            self.twist_commander.publish(self.twist_cmd)
            timer += self.sample_dt
            self.loop_timer.sleep()

        self.do_halt()

    def do_move(self, distance):
        timer = 0
        final_time = math.fabs(distance) / self.move_speed
        self.twist_cmd.angular.z = 0
        self.twist_cmd.linear.x = utils.sgn(distance) * self.move_speed

        while timer < final_time:
            self.twist_commander.publish(self.twist_cmd)
            timer += self.sample_dt
            self.loop_timer.sleep()

        self.do_halt()

    def do_halt(self):
        self.twist_cmd.angular.z = 0
        self.twist_cmd.linear.x = 0

        for i in range(0, 10):
            self.twist_commander.publish(self.twist_cmd)
            self.loop_timer.sleep()

    def callback(self, goal):
        rospy.loginfo("New pose path goal received")
        poses_list = goal.path.poses
        rospy.loginfo("New path has %d poses", len(poses_list))

        feedback = pose_pathFeedback()
        for i in range(len(poses_list)):
            pose_desired = poses_list[i].pose
            rospy.loginfo("Executing pose %d in goal path", i)
            feedback.pose = pose_desired
            feedback.completed.data = False
            self.server.publish_feedback(feedback)

            desired = self.get_yaw_and_dist(pose_desired)
            spin_angle = desired[0]
            travel_distance = desired[1]
            rospy.loginfo("Calculated desired yaw of %f", spin_angle)
            rospy.loginfo("Calculated desired travel distance of %f",
                           travel_distance)

            spin_angle = utils.min_spin(spin_angle)
            self.do_spin(spin_angle)
            self.current_pose.orientation = pose_desired.orientation

            self.do_move(travel_distance)
            # Current pose will always be (0,0,0)
            # Thus the position sent to the server will always be relative
            # to the current position, rather than an absolute coordinate
            feedback.completed.data = True
            self.server.publish_feedback(feedback)

        result = pose_pathResult(path=goal.path)
        self.server.set_succeeded(result)


def main():
    rospy.init_node('pose_path_server')
    server = ActionServer()
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
