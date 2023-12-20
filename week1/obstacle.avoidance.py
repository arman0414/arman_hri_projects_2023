import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

def avoidance_callback(scan_data):
    mini_distnace = min(scan_data.ranges)
    
    
    if mini_distnace < 1:
        msg_twist.linear.x = 0
    else: 
        msg_twist.linear.x =1
    
    if mini_distnace < 1.5:
        left_distance = sum(scan_data.ranges[:len(scan_data.ranges)//2])
        right_distance = sum(scan_data.ranges[:len(scan_data.ranges)//2])
        
        if left_distance < right_distance:
            msg_twist.angular.z = 0.15 #turns right
        else:
            msg_twist.angular.z = -0.15 #turns left
    else: 
        msg_twist.angular.z = 0 #No turning
    
    velocity_publisher.publish(msg_twist)
rospy.init_node('obstacle_avoidance_node')
laser_subscriber= rospy.Subscriber('/base_scan', LaserScan, avoidance_callback )
velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
msg_twist = Twist()

#running the node
rospy.spin()