#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PointStamped, TransformStamped
from tf2_ros import TransformListener, TransformBroadcaster, Buffer
from tf2_geometry_msgs import do_transform_point
from yolov8_ros_msgs.msg import BoundingBox, BoundingBoxes, PointDetection, PointDetectionArray
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2 
from scipy.optimize import linear_sum_assignment
import numpy as np

import cv2

class TrackingDetection:
    def __init__(self, point_detection):
        self.lost_cnt = 0
        self.observe_cnt = 1
        self.detection = point_detection
        self.is_valid = False

class DetectionTracker:
    def __init__(self):
        rospy.init_node('obstacle_tracker_node')
        
        # Initialize TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)

        # Initialize TF broadcaster
        self.tf_broadcaster = TransformBroadcaster()

        # Subscribe to PointDetectionArray with frame_id == base_link
        rospy.Subscriber("/yolo_detections_base", PointDetectionArray, self.detections_callback)

        # Publisher for transformed detections with frame_id == map
        self.detections_pub = rospy.Publisher("/yolo_detections_map", PointDetectionArray, queue_size=10)

        self.object_position_map_pub = rospy.Publisher('/object_position', PointStamped, queue_size=10)

        self.pc2_pub = rospy.Publisher('yolo_tracked_pc2', PointCloud2, queue_size=1)

        self.tracking_detections = {
            'cow': [],
            'robot': [],
            'manure': []
        }

        self.start_tracking_threshold = 2
        self.end_tracking_threshold = 3
        self.distance_thresold = 0.2 # 0.5 for debug association
        self.enable_visual = True

        if self.enable_visual:
            self.blank_image = np.ones((480, 480, 3), np.uint8) * 255
            cv2.arrowedLine(self.blank_image, (0, 240), (20, 240), (0, 0, 255), 1)

    def detections_callback(self, msg):
        print("detection received")
        try:
            self.tf_buffer.can_transform('map', 'base_link', rospy.Time(0))
        except:
            print("no transformation map->base_link available")
            return
        transformed_detections = []

        for detection in msg.detections:
            # Transform the detection point to map frame
            # print("here")
            transformed_point = self.transform_point(detection.point, msg.header.frame_id, "map")
        
            # Update the detection point with the transformed point
            transformed_detection = detection # same label and conf; TODO: update confidence with localization covariance
            transformed_detection.point = transformed_point.point
            transformed_detections.append(transformed_detection)

            # visualization
            self.object_position_map_pub.publish(transformed_point)

        # Publish transformed detections
        tracking_detections_msg = self.filter_detections(transformed_detections) 
        self.detections_pub.publish(tracking_detections_msg)

        # visualization
        if self.enable_visual:
            self.signal_decision(msg.detections, tracking_detections_msg, self.blank_image)

        print("transform ", len(tracking_detections_msg.detections), " points to map")

    def transform_point(self, point, src_frame, target_frame):
        # Convert PointStamped to TransformStamped
        point_stamped = PointStamped()
        point_stamped.header.stamp = rospy.Time(0) # in principle, this makes the self.tf_buffer.transform only look for the latest regardless timestamp of tf
        # but this only works when local ros thinks ros.Time.now() == 0, which is another error we should avoid :(
        point_stamped.header.frame_id = src_frame # need this for transformPoint() function
        point_stamped.point = point
        
        try:
            # Transform point to target frame
            transformed_point = self.tf_buffer.transform(point_stamped, target_frame, rospy.Duration(1000.0)) # sometimes good, sometimes error
            # it seems that sometimes the /tf published by the robot is delayed
            return transformed_point
        except Exception as e:
            rospy.logerr("Failed to transform point: %s", str(e))
            return None

    def filter_detections(self, transformed_detections): # not considering false positive across different labels
        # 1. check labels
        try:
            for detection in transformed_detections:
                assert detection.label == 'cow' or 'manure' or 'robot'
        except:
            print("Error! unwanted detection label: ", transformed_detection.label)
            exit(0)
        
        # 2. for non-empty trackings
        if len(self.tracking_detections['cow']) or len(self.tracking_detections['robot']) or len(self.tracking_detections['manure']) :
            # 2.1 sort detections by label
            new_detections = {
            'cow': [],
            'robot': [],
            'manure': []
            }
            for transformed_detection in transformed_detections:
                new_detections[transformed_detection.label].append(transformed_detection)
            
            # 2.2 iterate each label
            for label in ['cow', 'robot', 'manure']:

                # 2.2.1 assign new detections to old trackings
                new_old_pairs, associated_new_ind, associated_old_ind = self.find_associations(
                    new_detections[label], 
                    self.tracking_detections[label]
                )

                # 2.2.2 for those associated
                for new_i, old_i in new_old_pairs: # tuples in list
                    
                    # 2.2.2.1 compute the distance
                    dist = np.linalg.norm(
                        np.array([
                            new_detections[label][new_i].point.x - self.tracking_detections[label][old_i].detection.point.x,
                            new_detections[label][new_i].point.y - self.tracking_detections[label][old_i].detection.point.y
                        ])
                    )

                    # 2.2.2.2 close enough? update with new detection : label tracking as lost
                    if dist < self.distance_thresold:
                        self.tracking_detections[label][old_i].detection = new_detections[label][new_i]
                        self.tracking_detections[label][old_i].lost_cnt = 0
                        self.tracking_detections[label][old_i].observe_cnt += 1

                    else:
                        # not change the detection if lost
                        self.tracking_detections[label][old_i].lost_cnt += 1
                        self.tracking_detections[label][old_i].observe_cnt = 0

                # 2.2.3 for unassociated new detections
                unassociated_new_ind = [i for i in np.arange(len(new_detections[label])) if i not in associated_new_ind]
                for i in unassociated_new_ind:
                    self.tracking_detections[label].append(TrackingDetection(new_detections[label][i]))

                # 2.2.4 for unassociated old trackings
                unassociated_old_ind = [i for i in np.arange(len(self.tracking_detections[label])) if i not in associated_old_ind]
                for i in unassociated_old_ind:
                    self.tracking_detections[label][i].lost_cnt += 1


                # 2.2.5 update the status, make valid or remove invalid
                tracking_detections_label_new = []
                for i in range(len(self.tracking_detections[label])):
                    print("wtf: ", len(self.tracking_detections[label]))
                    if self.tracking_detections[label][i].lost_cnt >= self.end_tracking_threshold:
                        # self.tracking_detections[label].pop(i)
                        continue
                        
                    elif (not self.tracking_detections[label][i].is_valid) and self.tracking_detections[label][i].observe_cnt > self.start_tracking_threshold:
                        self.tracking_detections[label][i].is_valid = True

                    tracking_detections_label_new.append(self.tracking_detections[label][i])
                self.tracking_detections[label] = tracking_detections_label_new

                              

        # 3. for empty trackings  
        else:
            for transformed_detection in transformed_detections:
                self.tracking_detections[transformed_detection.label].append(TrackingDetection(transformed_detection))


        # 4. return the ROS msg
        tracking_detections_msg = PointDetectionArray()
        # transformed_detections.header.stamp = msg.header.stamp
        tracking_detections_msg.header.stamp = rospy.Time.now()
        tracking_detections_msg.header.frame_id = "map"

        for labeled_trackings in self.tracking_detections.values():
            for tracking_detection in labeled_trackings:
                if tracking_detection.is_valid:
                    tracking_detections_msg.detections.append(tracking_detection.detection)
                    
        return tracking_detections_msg

    def find_associations(self, new_detections, old_detections):
        num_new = len(new_detections)
        num_old = len(old_detections)
        cost_matrix = np.full( (num_new, num_old), np.inf )
        for i in range(num_new):
            for j in range(num_old):
                cost_matrix[i, j] = np.linalg.norm(
                    np.array([
                        new_detections[i].point.x - old_detections[j].detection.point.x,
                        new_detections[i].point.y - old_detections[j].detection.point.y
                    ])
                )

        new_ind, old_ind = linear_sum_assignment(cost_matrix)
        optimal_new_old_pairs = [(new_ind[i], old_ind[i]) for i in range(len(new_ind))] # no worrying num_new > len(new_ind)
        return optimal_new_old_pairs, new_ind, old_ind

    def signal_decision(self, raw_detections, tracking_detections_msg, blank_img):
        # TODO: visualize under base_link, or no point in figure!
        img = blank_img.copy() # deep?
        scale = 6 # [m]
        img_size = 480

        num_manure = 0
        manure_thresh = 1
        num_robot = 0
        robot_thresh = 1
        num_cow = 0
        cow_thresh = 1

        cow_and_robot_points = []

        # raw detections
        for i in range(len(raw_detections)):
            px = int(raw_detections[i].point.x/scale * img_size)
            py = int( - raw_detections[i].point.y/scale * img_size + img_size * 0.5)
            cv2.circle(img, (px, py), 5, (130, 130, 130), -1)  # red circle

        for i in range(len(tracking_detections_msg.detections)):
            tracking_point_map = PointStamped()
            tracking_point_map.header.stamp = rospy.Time(0) # in principle, this makes the self.tf_buffer.transform only look for the latest regardless timestamp of tf
            # but this only works when local ros thinks ros.Time.now() == 0, which is another error we should avoid :(
            tracking_point_map.header.frame_id = 'map' # need this for transformPoint() function
            tracking_point_map.point = tracking_detections_msg.detections[i].point

            tracking_point_baselink = self.tf_buffer.transform(tracking_point_map, 'base_link', rospy.Duration(1000.0))
            px = int(tracking_point_baselink.point.x/scale * img_size)
            py = int( - tracking_point_baselink.point.y/scale * img_size + img_size * 0.5)
            # cv2.circle(img, (px, py), 5, (255-50*i, 0+50*i, 0), -1)  # blue circle # test association with tracker_test.py
            
            # change color according to type, and make decisions
            if tracking_detections_msg.detections[i].label == 'manure': # purple
                dot_color = (252, 3, 165)
                if tracking_point_baselink.point.x ** 2 + tracking_point_baselink.point.y ** 2 <= manure_thresh ** 2:
                    print("manure close")

            elif tracking_detections_msg.detections[i].label == 'robot': # red
                dot_color = (0, 0, 255)
                cow_and_robot_points += self.generate_circle_points(
                    tracking_point_baselink.point.x,
                    tracking_point_baselink.point.y,
                    0.15
                )
                if tracking_point_baselink.point.x ** 2 + tracking_point_baselink.point.y ** 2 <= robot_thresh ** 2:
                    print("robot close")

            elif tracking_detections_msg.detections[i].label == 'cow': # green
                dot_color = (3, 252, 165)
                cow_and_robot_points += self.generate_circle_points(
                    tracking_point_baselink.point.x,
                    tracking_point_baselink.point.y,
                    0.10
                )
                if tracking_point_baselink.point.x ** 2 + tracking_point_baselink.point.y ** 2 <= cow_thresh ** 2:
                    print("cow close")
            
            cv2.circle(img, (px, py), 5, dot_color, -1)

        
        pc2_fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]

        pc2_header = tracking_detections_msg.header
        pc2_header.frame_id = "base_link"
        pc2_msg = pc2.create_cloud(header=pc2_header, fields=pc2_fields, points=cow_and_robot_points)
        self.pc2_pub.publish(pc2_msg)

        img = cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE)
        cv2.imshow("Bird Eye View", img)
        cv2.waitKey(3) # 3 ms

    
    def generate_circle_points(self, center_x, center_y, radius, num_points=10):
        points = []
        for i in range(num_points):
            theta = 2 * np.pi * i / num_points
            point_x = center_x + radius * np.cos(theta)
            point_y = center_y + radius * np.sin(theta)
            points.append([point_x, point_y, 0])
        return points


if __name__ == '__main__':
    try:
        detector = DetectionTracker()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass