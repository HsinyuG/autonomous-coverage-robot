#!/usr/bin/env python

import rospy
import sensor_msgs.point_cloud2 as pc2 
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointField
import numpy as np
# import laser_geometry.laser_geometry as lg

# TODO: find the orientation of scan of obstacles, filter them out
class Lidar_Preprocess:
    def __init__(self):
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback, queue_size=1)
        self.pc2_pub = rospy.Publisher('scan_filtered_pc2', PointCloud2, queue_size=1)
        # self.scan_pub = rospy.Publisher('filtered_scan', LaserScan, queue_size=1)

    def scan_callback(self, msg):
        filtered_points = self.filter_scan(msg)
        self.publish_filtered_points(filtered_points, msg.header)
        # self.publish_filtered_scan(filtered_points, msg.header)

    def filter_scan(self, msg):
        filtered_points = []
        for i, range_value in enumerate(msg.ranges):
            point_orientation_rad = msg.angle_min + msg.angle_increment * i
            if not self.is_point_occluded(range_value, point_orientation_rad):
                point_local_x = range_value * np.cos(point_orientation_rad)
                point_local_y = range_value * np.sin(point_orientation_rad)
                filtered_points.append([point_local_x, point_local_y, 0.0])
                # filtered_points.append([point_local_x, point_local_y])

        return filtered_points

    def is_point_occluded(self, distance, angle_rad):
        result = None
        angle_deg = angle_rad * 180 / np.pi
        # laser frame left x back y
        # 90-15=75, 90-25=65, 90+15=105, 90+25=115
        
        # pillar in the rear cornors
        if distance < 0.25 and ((65 < angle_deg <= 75) or (105 < angle_deg <= 115)):
            result = True
        # pillar under the onboard pc
        elif distance < 0.15 and ((45 < angle_deg <= 65) or (115 < angle_deg <= 135)):
            result = True
        # pillar in the front cornors
        # elif distance < 0.10 and ((-10 < angle_deg <= 45) or (135 < angle_deg <= 181) or (-181 < angle_deg <= -170)):
        elif distance < 0.10 and ((-30 < angle_deg <= 45) or (135 < angle_deg <= 181) or (-181 < angle_deg <= -150)):
            result = True
        else:
            result = False
            # if distance < 0.25:
            #     print("missing point:")
            #     print(angle_deg)
            #     print(distance)
        return result
            
        

    def publish_filtered_points(self, filtered_points, scan_header):

        pc2_fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]

        pc2_msg = pc2.create_cloud(header=scan_header, fields=pc2_fields, points=filtered_points)
        # pc2_msg.is_dense = True

        self.pc2_pub.publish(pc2_msg)

if __name__ == '__main__':
    rospy.init_node('lidar_preprocess')
    lidar_preprocess = Lidar_Preprocess()
    rospy.spin()







