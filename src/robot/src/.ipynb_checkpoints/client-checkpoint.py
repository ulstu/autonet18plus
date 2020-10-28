#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import division
import sys
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import cv2
import math
from cv_bridge import CvBridge, CvBridgeError
import datetime
import time


class ClientController:
    can_move = False

    def __init__(self):

        rospy.loginfo('start client')
        rospy.init_node('client_node', anonymous=True)
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        rospy.Subscriber("cmd", String, self.cmd_callback)
        rospy.Subscriber("cam_top", Image, self.img_top_callback)
        rospy.Subscriber("cam_bottom", Image, self.img_bottom_callback)
        self.bridge = CvBridge()
        self.run()

    def move(self, speed, angle, distance):
        cmd = Twist()
        cmd.linear.x = speed
        cmd.linear.y = 0
        cmd.linear.z = 0
        cmd.angular.x = 0
        cmd.angular.y = 0
        cmd.angular.z = angle
        rospy.loginfo('ros client spin. can_move: {}'.format(self.can_move))
        t0 = rospy.Time.now().to_sec()
        current_distance = 0

        r = rospy.Rate(10)
        while (current_distance < distance):
            self.cmd_vel_pub.publish(cmd)
            t1 = rospy.Time.now().to_sec()
            rospy.loginfo("{} {}".format(current_distance, distance))
            current_distance = abs(speed) * (t1 - t0)
            r.sleep()

        # After the loop, stops the robot
        cmd.linear.x = 0
        cmd.angular.z = 0

        # Force the robot to stop
        self.cmd_vel_pub.publish(cmd)
        rospy.sleep(0.8)


    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            try:
                if self.can_move:
                    self.move(0.7, 0, 2.2)
                    self.move(0.7, -1, 1.4)
                    self.move(0.7, 0, 5.5)
                    self.move(0.7, -1, 1.4)
                    self.move(0.7, 0, 4.)
                    self.move(0.7, -1, 1.4)
                    self.move(0.7, 0, 4)
                    self.move(0.7, 1, 1.5)
                    self.move(0.7, 0, 3)
                    self.move(0.7, 1, 1.4)
                    self.move(0.7, 0, 9)
                    self.move(0.7, -1, 1.4)
                    self.move(0.7, 0, 4)

                    self.can_move = False
            except:
                rospy.loginfo("Error {}".format(sys.exc_info()))
            r.sleep()
        rospy.loginfo("run method finished")

    def time_stamped(self, fname, fmt='%Y-%m-%d-%H-%M-%S-{fname}'):
        return datetime.datetime.now().strftime(fmt).format(fname=fname)

    def img_top_callback(self, img):
        #rospy.loginfo('top img received')
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img, "passthrough")
            cv2.imwrite('/mnt/d/{}'.format(self.time_stamped('img.png')), cv_image)
            #cv2.imshow('top image', cv_image)
        except CvBridgeError as e:
            print(e)

    def img_bottom_callback(self, img):
        #rospy.loginfo('bottom img received')
        #try:
        #    cv_image = self.bridge.imgmsg_to_cv2(img, "passthrough")
        #    cv2.imshow('bottom image', cv_image)
        #except CvBridgeError as e:
        #    print(e)
        pass

    def cmd_callback(self, data):
        data = str(data)
        if "START" in data:
            self.can_move = True
            rospy.loginfo("START message received by client")

        elif "STOP" in data or "INTERRUPT" in data:
            rospy.loginfo('motor stop')
            self.can_move = False

if __name__ == '__main__':
    try:
        ClientController()
    except rospy.ROSInterruptException:
        rospy.logerr('Ошибка client node: {}'.format(traceback.format_exc()))
