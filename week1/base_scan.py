import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

def callback(data):
    closest= data.ranges[0]

    for i in range(len(data.range)):
        if closest > data.range[i]:
            closest= data.range[i]
    print(closest)

def listener():
    rospy.init_node('base_scan_listener', anonymous=True)
    
    rospy.Subscriber('/base_scan', LaserScan, callback)
    #spin() Simply keeps python from existing untill this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()