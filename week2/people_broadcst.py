import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Transform_Stamped
from people_msgs.msg import PositionMeasurementArray
import tf2_ros
import tf_conversations

def people_callback(pmf):
    transform_broadcaster = tf2_ros.TranformCaster()
    for people in pmf.people:
        if people.header.frame_id =="odom":
            t = Transform_Stamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id ="odom"
            t.child_frame_id = "pframe"
            t.transform.translation.x = people.pos.x
            t.transform.translation.y = people.pos.y
            t.transform.translation.z = people.pos.z
            q= tf_conversations.transformation.quaternion_form_euler(0,0,0)
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.q = q[3]
            transform_broadcaster.sendTransform(t)
    
if __name__=="__main__":
    rospy.init_node("peroson_pos_caster")

    rospy.Subscriber("/people_tracker_measurement", PositionMeasurementArray, people_callback)
    rospy.spin()