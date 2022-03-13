#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import cv2
import message_filters


class CombineImages(Node):
    def __init__(self):
        super().__init__('image_combine')
        self.bridge = CvBridge()
        image_left = message_filters.Subscriber(self, Image, '/sitl_ssrc_fog_x_0/camera_left/image_raw')
        image_right = message_filters.Subscriber(self, Image, '/sitl_ssrc_fog_x_0/camera_right/image_raw')
        self.pub_img = self.create_publisher(Image, 'vr_image',1)
        ts = message_filters.TimeSynchronizer([image_left, image_right], 1)
        ts.registerCallback(self.callback)

        #ts = message_filters.ApproximateTimeSynchronizer([sub_center_camera, sub_left_camera, sub_right_camera, sub_control], 1, 0.1)


    def callback(self, image_left, image_right):
        left = self.bridge.imgmsg_to_cv2(image_left, "bgr8")
        right = self.bridge.imgmsg_to_cv2(image_right, "bgr8")
        img_concat = cv2.hconcat([left, right])
        try:
            self.pub_img.publish(self.bridge.cv2_to_imgmsg(img_concat, "bgr8"))
        except CvBridgeError as e:
            print(e)


def main(args=None):
    rclpy.init(args=args)
    image_combine_ = CombineImages()
    rclpy.spin(image_combine_)


if __name__ == '__main__':
    main()





