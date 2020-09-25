#!/usr/bin/env python

import rospy
from darknet_ros_msgs.msg import BoundingBoxes
from people_msgs.msg import People
from people_msgs.msg import Person
from geometry_msgs.msg import PoseArray, Pose
import names
import math


class people_watchtower():
    def __init__(self):
        self.people = People()
        self.angles = PoseArray()

        self.pub = rospy.Publisher(
            '/people_yolo_detector', People, queue_size=10)

        self.pub2 = rospy.Publisher(
            '/people_yolo_angles', PoseArray, queue_size=1)

        self.sub = rospy.Subscriber("/darknet_ros/bounding_boxes",
                                    BoundingBoxes, self.callback)

    def callback(self, data):

        self.people.header = data.header
        self.people.header.frame_id = "camera_rgb_optical_frame"

        self.angles.header = data.header

        person_list = data.bounding_boxes

        temp = []
        temp_angles = []

        for ppl in person_list:
            person = Person()
            pose = Pose()

            y = (ppl.xmax + ppl.xmin) / 2

            # # Position relative to world (center of camera is 90 degrees)
            # position_y = -0.090625*(y - 640) + 60

            # Position relative to robot (center of camera is 0 degrees)
            position_y = (y - 320) * 29 / 320

            position_y_rad = position_y * math.pi / 180

            # pose.position.x = 1
            pose.position.y = position_y_rad
            pose.orientation.w = 1

            person.position.y = position_y
            person.reliability = ppl.probability
            person.name = ppl.Class + "_" + names.get_first_name()

            temp_angles.append(pose)
            temp.append(person)

        self.people.people = temp
        self.angles.poses = temp_angles

        self.pub.publish(self.people)
        self.pub2.publish(self.angles)


if __name__ == '__main__':
    rospy.init_node('people_yolo_angle_detector', anonymous=True)
    people_watchtower()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
