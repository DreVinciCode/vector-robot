#!/usr/bin/env python3

import rospy

from std_msgs.msg import String, Int16, Float32, Bool
import keyboard

import anki_vector

from anki_vector_ros.msg import Dist, Pose, Color, Response


from behavior import Behavior




# self.response_pub = Publisher("/behavior/response", Response, queue_size=10)
# def drive_straight(self, msg):
#     dist = anki_vector.util.distance_mm(msg.distance)
#     speed = anki_vector.util.speed_mmps(msg.speed)
#     resp = self.robot.behavior.drive_straight(dist, speed)
#     self.publish_response(resp)

rospy.init_node('Vector_Teleop')

response_pub = rospy.Publisher("/behavior/drive_straight", Dist, queue_size=10)


while True:
    if keyboard.read_key() == 'i':
        print("i  pressed...")
        test = Dist()

        test.distance = float(100)
        test.speed = float(50)
        response_pub.publish(test)

    if keyboard.read_key() == 'j':
        print("j  pressed...")


