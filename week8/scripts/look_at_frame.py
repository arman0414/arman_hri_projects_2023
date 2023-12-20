#!/usr/bin/env python3
import rospy
import tf2_ros
from sensor_msgs.msg import JointState
from math import atan2, asin, sqrt

def interpolate_angles(start_angle, end_angle, fraction):
# Interpolate between start and end angles
    return start_angle + fraction * (end_angle - start_angle)

def calculate_head_angles(target, head_position):
# Calculate yaw and pitch angles to make the head look at the target
    dx = target.x - head_position.x
    dy = target.y - head_position.y
    dz = target.z - head_position.z

    distance = sqrt(dx**2 + dy**2 + dz**2)
    yaw = atan2(dy, dx)
    pitch = -asin(dz / distance)
    return yaw, pitch

def move_head_to_target():
    rospy.init_node('move_head_to_target')
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)
    joint_state_publisher = rospy.Publisher('/joint_states', JointState, queue_size=10)
    rate = rospy.Rate(10)

    current_yaw, current_pitch = 0.0, 0.0

    while not rospy.is_shutdown():
        try:
            # Look up transforms
            target_point_transform = tf_buffer.lookup_transform('base_link', 'pointing_frame', rospy.Time())
            head_transform = tf_buffer.lookup_transform('base_link', 'Head', rospy.Time())

        # Calculate target yaw and pitch angles
            target_yaw, target_pitch = calculate_head_angles(target_point_transform.transform.translation, head_transform.transform.translation)

    # Interpolate and publish intermediate joint states
            for i in range(0, 11):
                interp_yaw = interpolate_angles(current_yaw, target_yaw, i / 10.0)
                interp_pitch = interpolate_angles(current_pitch, target_pitch, i / 10.0)

                # Publish joint states
                joint_msg = JointState()
                joint_msg.header.stamp = rospy.Time.now()
                joint_msg.name = ['HeadYaw', 'HeadPitch']
                joint_msg.position = [interp_yaw, interp_pitch]
                joint_state_publisher.publish(joint_msg)
                rate.sleep()

# Update current angles
            current_yaw, current_pitch = target_yaw, target_pitch

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            continue

        rate.sleep()

if __name__ == '__main__':
     move_head_to_target()