import rospy
from geometry_msgs.msg import Transform_Stamped
import tf2_ros
import tf_conversations
from nav_msgs.msg import Odometry

def robot_callback(pmf):
            trans_broadcaster = tf2_ros.TranformCaster()
            t = Transform_Stamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id ="odom"
            t.child_frame_id = "rframe"
            t.transform.translation.x = pmf.pos.x
            t.transform.translation.y = pmf.pos.y
            t.transform.translation.z = pmf.pos.z
            t.transform.rotation.x = pmf.pose.pose.position.x
            t.transform.rotation.y = pmf.pose.pose.position.y
            t.transform.rotation.z = pmf.pose.pose.position.z
            t.transform.rotation.w = pmf.pose.pose.position.w
            trans_broadcaster.sendTransform(t)
    
if __name__=="__main__":
    rospy.init_node("robot_pos_caster")

    rospy.Subscriber("/base_pose_ground_truth", Odometry, robot_callback)
    rospy.spin()