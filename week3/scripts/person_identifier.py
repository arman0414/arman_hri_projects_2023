#!/usr/bin/env python3
import rospy
from people_msgs.msg import PositionMeasurementArray
from math import sqrt

DISTANCE_THRESHOLD = 1.0 # meters, for example

def compute_distance(person_a, person_b):
    dx = person_a.pos.x - person_b.pos.x
    dy = person_a.pos.y - person_b.pos.y
    return sqrt(dx**2 + dy**2)

def handle_people_measurements(data):
    detected_people = data.people
    n_people = len(detected_people)

# Check each pair of detected people to see if they are close enough
    for i in range(n_people):
        for j in range(i + 1, n_people):
            if compute_distance(detected_people[i], detected_people[j]) < DISTANCE_THRESHOLD:
              rospy.loginfo(f"Persons {detected_people[i].id} and {detected_people[j].id} are close to each other.")

def run_people_distance_detector():
    rospy.init_node('distance_detector_node')

    rospy.Subscriber('/people_tracker_measurements', PositionMeasurementArray, handle_people_measurements)

    rospy.spin()

if __name__ == '__main__':
    run_people_distance_detector()