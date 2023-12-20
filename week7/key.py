import rospy
from sensor_msgs.msg import JointState

key_poses = ["HeadYaw", "HeadPitch", "LHipYawPitch", "LHipRoll", "LHipPitch", "LKneePitch",
"LAnklePitch", "LAnkleRoll", "RHipYawPitch", "RHipRoll", "RHipPitch", "RKneePitch",
"RAnklePitch", "RAnkleRoll", "LShoulderPitch", "LShoulderRoll", "LElbowYaw",
"LElbowRoll", "LWristYaw", "LHand", "RShoulderPitch", "RShoulderRoll", "RElbowYaw",
"RElbowRoll", "RWristYaw", "RHand", "RFinger23", "RFinger13", "RFinger12", "LFinger21",
"LFinger13", "LFinger11", "RFinger22", "LFinger22", "RFinger21", "LFinger12", "RFinger11",
"LFinger23", "LThumb1", "RThumb1", "RThumb2", "LThumb2"]
def interpolate_poses(pose1, pose2, fraction):
    interpolated_pose = {}
    for joint in pose1:
        interpolated_pose[joint] = pose1[joint] + fraction * (pose2[joint] - pose1[joint])
        return interpolated_pose

def animate_nao():
    rospy.init_node('nao_animator')
    pub = rospy.Publisher('/joint_states', JointState, queue_size=10)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        for i in range(len(key_poses) - 1):
            start_pose = key_poses[i]
            end_pose = key_poses[i + 1]
            for t in range(11): # Increase the range to 11 for inclusive end
                fraction = t / 10.0
                interpolated_pose = interpolate_poses(start_pose, end_pose, fraction)

                joint_state_msg = JointState()
                joint_state_msg.header.stamp = rospy.Time.now() + rospy.Duration(t * 0.1) # Use rospy.Duration for timing
                for joint, position in interpolated_pose.items():
                    joint_state_msg.name.append(joint)
                    joint_state_msg.position.append(position)

                pub.publish(joint_state_msg)
                rate.sleep()

if __name__ == '__main__':
    try:
        animate_nao()
    except rospy.ROSInterruptException:
        pass