#!/usr/bin/env python3
import rospy
from std_msgs.msg import String, Bool
from sensor_msgs.msg import JointState
from math import atan2, asin, sqrt, pi
import threading

class RobotController:
    def __init__(self):
        rospy.init_node('robot_controller', anonymous=True)
        self.joint_states_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        self.speech_sub = rospy.Subscriber("/speech_recognition/final_result", String, self.speech_callback)
        self.tts_status_pub = rospy.Publisher('/tts/status', Bool, queue_size=10)
        self.tts_status_pub.publish(False)
        self.key_poses = [
        {"HeadYaw": 0, "HeadPitch": 0},
        {"HeadYaw": pi/4, "HeadPitch": pi/4},
        ]

    def speech_callback(self, data):
        rospy.loginfo("Received speech: %s", data.data)
        text_received = data.data
        if text_received == "move":
            self.move()

    def interpolate_poses(self, pose1, pose2, fraction):
        interpolated_pose = {}
        for joint in pose1:
            interpolated_pose[joint] = pose1[joint] + fraction * (pose2[joint] - pose1[joint])
        return interpolated_pose

    def move(self):
        rate = rospy.Rate(10)
        for i in range(len(self.key_poses) - 1):
            start_pose = self.key_poses[i]
            end_pose = self.key_poses[i + 1]
            for t in range(10):
                fraction = t / 10.0
                interpolated_pose = self.interpolate_poses(start_pose, end_pose, fraction)

                joint_state_msg = JointState()
                joint_state_msg.header.stamp = rospy.Time.now()
                joint_state_msg.name = list(interpolated_pose.keys())
                joint_state_msg.position = list(interpolated_pose.values())

                self.joint_states_pub.publish(joint_state_msg)
                rate.sleep()

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    controller = RobotController()
    controller.run()