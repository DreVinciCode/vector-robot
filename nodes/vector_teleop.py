#!/usr/bin/env python3

from rospy import Subscriber, Publisher
from std_msgs.msg import String, Int16, Float32, Bool
import keyboard
import rospy

import anki_vector

from behavior import Behavior

from anki_vector_ros.msg import Dist, Pose, Color, Response



# self.response_pub = Publisher("/behavior/response", Response, queue_size=10)
# def drive_straight(self, msg):
#     dist = anki_vector.util.distance_mm(msg.distance)
#     speed = anki_vector.util.speed_mmps(msg.speed)
#     resp = self.robot.behavior.drive_straight(dist, speed)
#     self.publish_response(resp)

rospy.init_node('Vector_Teleop')


while True:
    if keyboard.read_key() == 'i':
        print("i  pressed...")

    if keyboard.read_key() == 'j':
        print("j  pressed...")


