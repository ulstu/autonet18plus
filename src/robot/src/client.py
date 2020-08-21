#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import division
import sys
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import datetime


class ClientController:
    can_move = False

    def __init__(self):

        rospy.loginfo('start client')
        rospy.init_node('client_node', anonymous=True)
        cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        rospy.Subscriber("cmd", String, self.cmd_callback)
        rospy.Subscriber("cam_top", Image, self.img_top_callback)
        rospy.Subscriber("cam_bottom", Image, self.img_bottom_callback)
        self.bridge = CvBridge()
        self.run()

    def run(self):
        r = rospy.Rate(1)
        while not rospy.is_shutdown():
            if self.can_move:
                try:
                    pass
                except:
                    rospy.error("Error {}".format(sys.exc_info()[0]))
            rospy.loginfo('ros client spin. can_move: {}'.format(self.can_move))
            r.sleep()

    def time_stamped(self, fname, fmt='%Y-%m-%d-%H-%M-%S-{fname}'):
        return datetime.datetime.now().strftime(fmt).format(fname=fname)

    def img_top_callback(self, img):
        rospy.loginfo('top img received')
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img, "passthrough")
            cv2.imwrite('/mnt/d/{}'.format(self.time_stamped('img.png')), cv_image)
            #cv2.imshow('top image', cv_image)
        except CvBridgeError as e:
            print(e)

    def img_bottom_callback(self, img):
        rospy.loginfo('bottom img received')
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
        elif "STOP" in data or "INTERRUPT" in data:
            rospy.loginfo('motor stop')
            self.can_move = False

if __name__ == '__main__':
    try:
        ClientController()
    except rospy.ROSInterruptException:
        rospy.logerr('Ошибка client node: {}'.format(traceback.format_exc()))
