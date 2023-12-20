#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import tf2_ros
import math

class ObstacleAvoidance:
    def __init__(self):
        rospy.init_node('robot_obstacle_avoidance', anonymous=True)
        rospy.Subscriber('/base_scan', LaserScan, self.avd_obstacle)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.twist_msg = Twist()
        self.min_distance = 0.5
        self.turn_threshold = 0.5

    def avd_obstacle(self, data):
        self.human()

        front_distance = min(data.ranges[0:45] + data.ranges[-45:])
        if front_distance < self.min_distance:
            self.twist_msg.linear.x = 0.0
            self.twist_msg.angular.z = 0.0
        else:
            self.twist_msg.linear.x = 0.5

            left_distance = min(data.ranges[0:90])
            right_distance = min(data.ranges[-90:])

            if left_distance < right_distance:
                self.twist_msg.angular.z = 0.5
            else:
                self.twist_msg.angular.z = -0.5


        self.pub.publish(self.twist_msg)

    def human(self):
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)

        robot_vel = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
        rate = rospy.Rate(10.0)

        while not rospy.is_shutdown():
            try:
                trans = tfBuffer.lookup_transform('robot_frame', 'person_frame', rospy.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rate.sleep()
                rospy.loginfo(e)
                continue

            msg = Twist()

            msg.angular.z = 4 * math.atan2(trans.transform.translation.y, trans.transform.translation.x)
            msg.linear.x = 0.5 * math.sqrt(trans.transform.translation.x ** 2 + trans.transform.translation.y ** 2)

            robot_vel.publish(msg)

            rate.sleep()





    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        obstacle_avoidance = ObstacleAvoidance()
        obstacle_avoidance.run()
    except rospy.ROSInterruptException:
        pass

    #took help from one of the classmate was unable to do