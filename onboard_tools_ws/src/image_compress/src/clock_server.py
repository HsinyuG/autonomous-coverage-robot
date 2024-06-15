#!/usr/bin/env python3

import rospy
from rosgraph_msgs.msg import Clock

def clock_publisher():
    rospy.init_node('clock_publisher', anonymous=True)
    rate = rospy.Rate(50)  # Publish rate (1 Hz in this example)

    clock_pub = rospy.Publisher('/clock', Clock, queue_size=10)

    while not rospy.is_shutdown():
        current_time = rospy.Time.now()
        clock_msg = Clock()
        clock_msg.clock = current_time
        clock_pub.publish(clock_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        clock_publisher()
    except rospy.ROSInterruptException:
        pass