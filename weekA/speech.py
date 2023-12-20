#!/usr/bin/env python3
import rospy
from std_msgs.msg import String, Bool

class SpeechRepeater:
    def __init__(self):
        rospy.init_node('speech_repeater', anonymous=True)
        self.speech_sub = rospy.Subscriber("/speech_recognition/final_result", String, self.speech_callback)
        self.tts_status_pub = rospy.Publisher('/tts/status', Bool, queue_size=10)
        self.tts_status_pub.publish(False)

    def speech_callback(self, data):
        rospy.loginfo("Received speech: %s", data.data)
        self.repeat_speech(data.data)

    def repeat_speech(self, text_received):
        pub = rospy.Publisher('/tts/phrase', String, queue_size=10)
        pub.publish(text_received)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    speech_repeater = SpeechRepeater()
    speech_repeater.run()