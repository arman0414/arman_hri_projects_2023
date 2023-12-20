import rospy
from geometry_msgs.msg import Twist

def talker():
    rospy.init_node('talker', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(5) #setting up the frequency to 5HZ for faster motion

    msg_twist = Twist()
    linear_speed = 1.0 #1m/s
    angular_speed = 1.0

    # this is the main loop for moving forward and turn while ROS is not shutting down
    while not rospy.is_shutdown():
        for _ in range(5):
            msg_twist.linear.x = linear_speed
            msg_twist.angular.z= 0

            pub.publish(msg_twist)
            rate.sleep()
        for _ in range(8):
            msg_twist.linear.x = 0
            msg_twist.angular.z = angular_speed
            pub.publish(msg_twist)
            rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptExecution:
        pass