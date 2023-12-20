#!/usr/bin/env python3
import rospy
import tf2_ros
import geometry_msgs.msg

def publish_custom_frame():
    rospy.init_node('publish_custom_frame')
    broadcaster = tf2_ros.TransformBroadcaster()
    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        custom_transform = geometry_msgs.msg.TransformStamped()

    # Customize the frame names and transform values
    custom_transform.header.frame_id = "CustomLinkA"
    custom_transform.header.stamp = rospy.Time.now()
    custom_transform.child_frame_id = "CustomLinkB"
    custom_transform.transform.translation.x = 2.0
    custom_transform.transform.rotation.z = 1.0

    broadcaster.sendTransform(custom_transform)
    rate.sleep()

if __name__ == '__main__':
    publish_custom_frame()