#!/usr/bin/env python3

# Software License Agreement (BSD License)
#
# Copyright (c) 2023, Michael Ferguson
# Copyright (c) 2016, Martin Guenther
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

'''This is a converter for the Intel Research Lab SLAM dataset
   ( http://kaspar.informatik.uni-freiburg.de/~slamEvaluation/datasets/intel.clf )
   to rosbag2'''

from math import cos, pi, sin
from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.serialization import serialize_message
import rosbag2_py
from sensor_msgs.msg import LaserScan
import sys
from tf2_msgs.msg import TFMessage


class ClfConverter:

    def make_tf_msg(self, x, y, theta, stamp):
        trans = TransformStamped()
        trans.header.stamp = stamp.to_msg()
        trans.header.frame_id = '/odom'
        trans.child_frame_id = '/base_link'
        trans.transform.translation.x = x
        trans.transform.translation.y = y
        trans.transform.rotation.x = 0.0
        trans.transform.rotation.y = 0.0
        trans.transform.rotation.z = sin(theta / 2.0)
        trans.transform.rotation.w = cos(theta / 2.0)

        msg = TFMessage()
        msg.transforms.append(trans)
        return msg

    def convert(self, input_name, output_name):
        # Create bag file
        writer = rosbag2_py.SequentialWriter()
        writer.open(
            rosbag2_py.StorageOptions(uri=output_name, storage_id='mcap'),
            rosbag2_py.ConverterOptions('', '')
        )
        writer.create_topic(
            rosbag2_py._storage.TopicMetadata(
                name='tf',
                type='tf2_msgs/msg/TFMessage',
                serialization_format='cdr'
            )
        )
        writer.create_topic(
            rosbag2_py._storage.TopicMetadata(
                name='scan',
                type='sensor_msgs/msg/LaserScan',
                serialization_format='cdr'
            )
        )

        # Fill bag file
        with open(input_name) as dataset:
            for line in dataset.readlines():
                line = line.strip()
                tokens = line.split(' ')
                if len(tokens) <= 2 or tokens[0] == '#':
                    continue
                
                command = tokens[0]
                stamp = rclpy.time.Time(seconds=float(tokens[-3]))
                if command == 'FLASER':
                    msg = LaserScan()
                    num_scans = int(tokens[1])

                    if num_scans != 180 or len(tokens) < num_scans + 9:
                        rospy.logwarn("unsupported scan format")
                        continue

                    msg.header.frame_id = 'base_link'
                    msg.header.stamp = stamp.to_msg()
                    msg.angle_min = -90.0 / 180.0 * pi
                    msg.angle_max = 90.0 / 180.0 * pi
                    msg.angle_increment = pi / num_scans
                    msg.time_increment = 0.2 / 360.0
                    msg.scan_time = 0.2
                    msg.range_min = 0.001
                    msg.range_max = 50.0
                    msg.ranges = [float(r) for r in tokens[2:(num_scans + 2)]]
                    writer.write('scan', serialize_message(msg), stamp.nanoseconds)

                    odom_x, odom_y, odom_theta = [float(r) for r in tokens[(num_scans + 2):(num_scans + 5)]]
                    tf_msg = self.make_tf_msg(odom_x, odom_y, odom_theta, stamp)
                    writer.write('tf', serialize_message(tf_msg), stamp.nanoseconds)

                elif command == 'ODOM':
                    odom_x, odom_y, odom_theta = [float(t) for t in tokens[1:4]]
                    tf_msg = self.make_tf_msg(odom_x, odom_y, odom_theta, stamp)
                    writer.write('tf', serialize_message(tf_msg), stamp.nanoseconds)


if __name__ == '__main__':
    converter = ClfConverter()
    converter.convert(sys.argv[1], sys.argv[2])
    