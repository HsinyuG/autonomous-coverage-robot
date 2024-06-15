#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, CompressedImage
import cv2
from cv_bridge import CvBridge
import numpy as np
from time import time_ns
# from std_msgs.msg import Empty
from std_msgs.msg import Header

class ImageResizer:
    def __init__(self):
        self.bridge = CvBridge()
        self.latest_image_msg = None
        self.timer = rospy.Timer(rospy.Duration(0.3), self.compress_publish)

        # Subscribe to the raw image topic
        rospy.Subscriber('camera/color/image_raw', Image, self.image_callback, queue_size=1, buff_size=52428800)

        # Publisher for the resized and compressed image
        self.resized_compressed_pub = rospy.Publisher('/resized/compressed', CompressedImage, queue_size=1)
        # self.empty_pub = rospy.Publisher('empty_topic', Empty, queue_size=10)

    def compress_publish(self, event):
        try:
            time_start = time_ns()
            image = self.latest_image_msg
            # Convert ROS Image to OpenCV format
            # cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            cv_image = np.frombuffer(image.data, dtype=np.uint8).reshape(image.height, image.width, -1)

            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            # Resize the image
            resized_image = cv2.resize(cv_image, (320, 320))

            # Convert the resized image back to ROS format
            # resized_image_msg = self.bridge.cv2_to_compressed_imgmsg(resized_image)

            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 50]  # Set quality here
            _, buffer = cv2.imencode('.jpg', resized_image, encode_param)
            compressed_image = buffer.tobytes()

            # Create CompressedImage message
            compressed_image_msg = CompressedImage()


            # Publish the resized and compressed image
            time_end = time_ns()
            secs = time_end // 1_000_000_000
            nsecs = time_end % 1_000_000_000
            header = Header()
            header.stamp = rospy.Time(secs, nsecs)
            # resized_image_msg.header = header
            compressed_image_msg.header = header


            compressed_image_msg.format = "jpeg"
            compressed_image_msg.data = np.array(compressed_image).tostring()

            # Publish the resized and compressed image
            self.resized_compressed_pub.publish(compressed_image_msg)


            


            print("compress time ns = ", time_end - time_start)
            # self.resized_compressed_pub.publish(resized_image_msg)
        except Exception as e:
            rospy.logerr(e)


    def image_callback(self, image_msg):
        # print("image")
        # empty_msg = Empty()
        # self.empty_pub.publish(empty_msg)
        # return
        self.latest_image_msg = image_msg

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    print("started")
    rospy.init_node('image_resizer_node')
    image_resizer = ImageResizer()
    image_resizer.run()
