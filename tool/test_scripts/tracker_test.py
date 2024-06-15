#!/usr/bin/env python
import os
os.system("bash -c 'source /home/xaviergg/Desktop/RO_MDP/group13/devel/setup.bash'")

import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import Point
from yolov8_ros_msgs.msg import PointDetection, PointDetectionArray
import random
import numpy as np
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

import cv2
import copy

print("1")

def add_noise(point):
    noise = np.random.normal(0, 0.1, 3)  # Mean = 0, standard deviation = 0.1, 3 dimensions (x, y, z)
    noisy_point = Point()
    noisy_point.x = point.x + noise[0]
    noisy_point.y = point.y + noise[1]
    noisy_point.z = point.z + noise[2]
    return noisy_point

def publish_yolo_detections():
    rospy.init_node('yolo_detections_publisher', anonymous=True)
    print("2")
    pub = rospy.Publisher('/yolo_detections_base', PointDetectionArray, queue_size=10)
    print("2")
    transform_pub = TransformBroadcaster()

    print("2")

    # rate = rospy.Rate(3)  # 3 Hz

    detections = []
    num = 5
    # Populate detections array
    for i in range(num):  # Assuming there are 5 detections
        random.seed(7*i)
        detection = PointDetection()

        # Assign a random point
        detection.point = Point(random.uniform(0, 5), random.uniform(-2.5, 2.5), random.uniform(-0.2, 0.1))
        print("x ", detection.point.x)
        print("y ", detection.point.y)

        # Add noise to the point
        # detection.point = add_noise(detection.point)

        # Assign label and confidence
        detection.label = 'cow' if i%2 else 'robot'
        detection.confidence = 0.8

        detections.append(detection)
    print(len(detections), "haha")

    print("3")

    height, width = 480, 480  # Define the size of the image
    blank_image = np.ones((height, width, 3), np.uint8) * 255
    cv2.arrowedLine(blank_image, (0, 240), (20, 240), (0, 0, 255), 1)

    def callback(event):
        print("4.1")
        # Create PointDetectionArray message
        detections_msg = PointDetectionArray()

        # Populate header
        detections_msg.header = Header()
        detections_msg.header.frame_id = 'base_link'

        # Populate image header
        detections_msg.image_header = Header()
        detections_msg.image_header.frame_id = 'base_link'

        noisy_detection_list = copy.deepcopy(detections)
        for detection in noisy_detection_list:
            print("before add", detection.point.x)
            detection.point = add_noise(detection.point)
            print("after add", detection.point.x)

        detections_msg.detections = noisy_detection_list

        # Publish detections
        pub.publish(detections_msg)

        print("4.2")

        # Publish a uniform transformation from 'map' to 'base_link'
        transform_msg = TransformStamped()
        transform_msg.header.stamp = rospy.Time.now()
        transform_msg.header.frame_id = 'map'
        transform_msg.child_frame_id = 'base_link'
        transform_msg.transform.translation.x = 0
        transform_msg.transform.translation.y = 0
        transform_msg.transform.translation.z = 0
        transform_msg.transform.rotation.w = 1.0  # No rotation
        transform_pub.sendTransform(transform_msg)

        img = blank_image.copy() # deep?
        # img = np.ones((height, width, 3), np.uint8) * 255
        for i in range(num):
            px = noisy_detection_list[i].point.x/5 * width
            py = noisy_detection_list[i].point.y/5 * height + 0.5 * height
            print("px: ", px)
            print("py: ", py)
            px = int(px)
            py = int(py)
            print("pxi: ", px)
            print("pyi: ", py)
            cv2.circle(img, (px, py), 5, (255-50*i, 0+50*i, 0), -1)  # Blue circle

        img = cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE)
        cv2.imshow("window", img)

        cv2.waitKey(3) # 3 ms

    timer = rospy.Timer(rospy.Duration(0.333), callback)
    print("4")

    rospy.spin()
    print("5")
        

if __name__ == '__main__':
    try:
        publish_yolo_detections()
    except rospy.ROSInterruptException:
        pass
