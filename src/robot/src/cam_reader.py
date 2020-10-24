#!/usr/bin/env python3
# -*- coding: utf-8 -*-

'''#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from cv_bridge.boost.cv_bridge_boost import getCvType
import cv2


class CameraReader:
    def __init__(self):
        rospy.init_node('camera_reader_node', anonymous=True)
        rospy.Subscriber("cam_bottom", Image, self.callback_bottom)
        rospy.Subscriber("cam_top", Image, self.callback_top)
        self.bridge = CvBridge()

        rospy.spin()


    def callback_top(self, data):
        # rospy.loginfo(rospy.get_caller_id() + " %s", data)
        # angles, ranges = self.prepare_data(data)
        camera_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        cv2.imwrite('/home/nvidia/projects/autonet18plus/src/robot/top.jpg', camera_img)

        # cv2.imshow('top', camera_img)

    def callback_bottom(self, data):
        # rospy.loginfo(rospy.get_caller_id() + " %s", data)
        # angles, ranges = self.prepare_data(data)
        camera_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        cv2.imwrite('/home/nvidia/projects/autonet18plus/src/robot/bot.jpg', camera_img)
        rospy.loginfo(camera_img.shape)
        # cv2.imshow('bot', camera_img)

if __name__ == '__main__':
    try:
        CameraReader()
    except rospy.ROSInterruptException:
        rospy.logerr('Ошибка CameraReader node: {}'.format(traceback.format_exc()))
