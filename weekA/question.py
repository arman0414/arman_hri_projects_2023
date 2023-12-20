#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

class Questionnaire:
    def __init__(self):
        rospy.init_node('speech_repeater', anonymous=True)
        self.stt_sub_node = "/speech_recognition/final_result"
        self.questions = [
        "Do you have an interest in data structures?",
        "Are algorithms something you enjoy?",
        "Do you possess any prior knowledge about time complexity?"
        "Do you like it?"
        ]
        self.question_state = 0
        self.user_feedback = []
        self.speech_sub = rospy.Subscriber(self.stt_sub_node, String, self.speech_callback)
        self.ask_and_process_question()

    def process_user_feedback(self):
        yes_count = sum("yes" in fb for fb in self.user_feedback)
        self.user_feedback.clear()
        self.question_state = 0

        if yes_count == 0:
            rospy.loginfo("It seems like you don't have a strong inclination towards computer science.")
        elif yes_count == 3:
            rospy.loginfo("It seems like you have a keen interest in computer science!")
        else:
            rospy.loginfo("It seems like you have some interest in computer science.")

            rospy.loginfo("#############Let's START AGAIN###################")
            self.ask_and_process_question()

    def ask_and_process_question(self):
            rospy.loginfo(self.questions[self.question_state])
            self.question_state += 1

    def speech_callback(self, data):
        response = data.data
        rospy.loginfo("We heard: %s", response)
        if "yes" in response or "no" in response:
            self.user_feedback.append(response)
            if self.question_state < 3:
                self.ask_and_process_question()
            else:
                self.process_user_feedback()
        else:
            rospy.loginfo("Please respond with 'YES' or 'NO'.")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    questionnaire = Questionnaire()
    questionnaire.run()