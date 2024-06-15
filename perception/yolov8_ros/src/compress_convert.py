#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np

class ImageResizer:
    def __init__(self):
        rospy.init_node('image_resizer_node', anonymous=True)
        self.bridge = CvBridge()
        self.latest_image_msg = None
        self.timer = rospy.Timer(rospy.Duration(0.3), self.compress_publish)

        # Subscribe to the raw image topic
        rospy.Subscriber('camera/color/image_raw', Image, self.image_callback, queue_size=1, buff_size=52428800)

        # Publisher for the resized and compressed image
        self.resized_compressed_pub = rospy.Publisher('camera/resized/compressed', CompressedImage, queue_size=10)


    def compress_publish(self, event):
        try:
            image = self.latest_image_msg
            # Convert ROS Image to OpenCV format
            # cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            cv_image = np.frombuffer(image.data, dtype=np.uint8).reshape(image.height, image.width, -1)

            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            # Resize the image
            resized_image = cv2.resize(cv_image, (320, 240))
            # Convert the resized image back to ROS format
            resized_image_msg = self.bridge.cv2_to_compressed_imgmsg(resized_image)
            # Publish the resized and compressed image
            print("publishing compressed image")
            self.resized_compressed_pub.publish(resized_image_msg)
        except Exception as e:
            rospy.logerr(e)


    def image_callback(self, image_msg):
        self.latest_image_msg = image_msg

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        image_resizer = ImageResizer()
        image_resizer.run()
    except rospy.ROSInterruptException:
        pass
