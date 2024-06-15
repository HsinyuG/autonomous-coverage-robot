#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Point
from tf.transformations import euler_from_quaternion
import math

global_plan = []
global_plan_received = False
current_waypoint_index = 0
last_waypoint_change_time = 0
distance_threshold = 0.35
orientation_thresh = 25 # degree
current_pose = None
starting_pos = None
reached_first_waypoint = False

def path_callback(msg):
    global global_plan, global_plan_received
    global_plan = msg.poses
    global_plan_received = True
    rospy.loginfo("Glob received")

def amcl_pose_callback(msg):
    global current_pose
    current_pose = msg.pose.pose

def starting_pos_callback(msg):
    global starting_pos
    starting_pos = msg

def get_distance(pose1, pose2):
    return math.hypot(pose1.position.x - pose2.position.x, pose1.position.y - pose2.position.y)

def get_angle_diff(pose1, pose2):
    orientation1 = pose1.orientation
    orientation2 = pose2.orientation

    # Convert quaternion to euler angles
    _, _, yaw1 = euler_from_quaternion([orientation1.x, orientation1.y, orientation1.z, orientation1.w])
    _, _, yaw2 = euler_from_quaternion([orientation2.x, orientation2.y, orientation2.z, orientation2.w])
    
    # Angular difference
    angle_diff = yaw2 - yaw1
    angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))  

    angle_diff_degrees = math.degrees(angle_diff)

    return angle_diff_degrees


def main():
    global current_waypoint_index, last_waypoint_change_time, current_pose, starting_pos, reached_first_waypoint
    
    rospy.init_node('path_subscriber', anonymous=True)
    rospy.Subscriber('/coverage_path/path0', Path, path_callback)
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, amcl_pose_callback)
    rospy.Subscriber('/area_division/robot0_starting_pos', Point, starting_pos_callback)
    goal_publisher = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
    
    rate = rospy.Rate(10) # 10 Hz
    while not rospy.is_shutdown():
        rospy.loginfo("glob")

        print(" =========================== current goal", current_waypoint_index)
       
        if global_plan_received and global_plan and current_pose and starting_pos:
            print(" =========================== inside if for goal", current_waypoint_index)

            # if not reached_first_waypoint:
            #     distance_to_start = math.hypot(current_pose.position.x - starting_pos.x, current_pose.position.y - starting_pos.y)
            #     if distance_to_start < distance_threshold:
            #         rospy.loginfo("Reached the starting position. Proceeding to the first waypoint.")
            #         reached_first_waypoint = True
            #         current_waypoint_index = 0
            
            # if reached_first_waypoint:
            distance_to_waypoint = get_distance(current_pose, global_plan[current_waypoint_index].pose)
            diff_deg_to_waypoint = get_angle_diff(current_pose, global_plan[current_waypoint_index].pose)
            print(" =========================== distance to goal", distance_to_waypoint)
            print(" =========================== angle diff to goal", diff_deg_to_waypoint)
            
            if distance_to_waypoint < distance_threshold : # and diff_deg_to_waypoint < orientation_thresh: # or (rospy.Time.now() - last_waypoint_change_time).toSec() > 7.0:
                current_waypoint_index += 1
                # last_waypoint_change_time = rospy.Time.now()
                if current_waypoint_index >= len(global_plan):
                    rospy.loginfo("All waypoints visited. Looping back to the start.")
                    current_waypoint_index = 0
                
            # next_waypoint_index = (current_waypoint_index + 1) % len(global_plan)
            goal = global_plan[current_waypoint_index].pose

            rospy.logdebug("Starting planning from (%.2f, %.2f) to (%.2f, %.2f)", current_pose.position.x, current_pose.position.y, goal.position.x, goal.position.y)
            
            goal_msg = PoseStamped()
            goal_msg.header.frame_id = "map"  # or your appropriate frame
            goal_msg.header.stamp = rospy.Time.now()
            goal_msg.pose = goal
            goal_publisher.publish(goal_msg)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
