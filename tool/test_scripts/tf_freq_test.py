#!/usr/bin/env python
#!/usr/bin/env python3

import rospy
import tf2_ros
import time
from collections import deque

class TransformRateChecker:
    def __init__(self):
        rospy.init_node('transform_rate_checker')

        self.frame_id = rospy.get_param('~frame_id', 'odom')
        self.child_frame_id = rospy.get_param('~child_frame_id', 'base_link')
        self.rate_check_interval = rospy.get_param('~rate_check_interval', 10.0)  # seconds
        self.buffer_size = int(self.rate_check_interval * 100)  # assuming 100 Hz max rate

        self.timestamps = deque(maxlen=self.buffer_size)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.rate_check_timer = rospy.Timer(rospy.Duration(self.rate_check_interval), self.check_rate)

        rospy.loginfo(f"Checking update rate of {self.frame_id}->{self.child_frame_id} every {self.rate_check_interval} seconds")

    def check_rate(self, event):
        now = rospy.Time.now()
        while self.timestamps and self.timestamps[0] < now - rospy.Duration(self.rate_check_interval):
            self.timestamps.popleft()

        num_transforms = len(self.timestamps)
        rate = num_transforms / self.rate_check_interval
        rospy.loginfo(f"Update rate of {self.frame_id}->{self.child_frame_id}: {rate:.2f} Hz")

    def run(self):
        rate = rospy.Rate(100)  # check for new transforms at 100 Hz
        while not rospy.is_shutdown():
            try:
                transform = self.tf_buffer.lookup_transform(self.frame_id, self.child_frame_id, rospy.Time(0))
                self.timestamps.append(rospy.Time.now())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                pass

            rate.sleep()

if __name__ == '__main__':
    checker = TransformRateChecker()
    checker.run()
