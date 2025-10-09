# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
import VL53L0X 
from sensor_msgs.msg import Range
from std_msgs.msg import Header
import time


from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Range, 'rangeLidar', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        
        # Create a VL53L0X object
        self.tof = VL53L0X.VL53L0X(i2c_bus=1,i2c_address=0x29)
        # I2C Address can change before tof.open()
        # tof.change_address(0x32)
        self.tof.open()
        # Start ranging
        self.tof.start_ranging(VL53L0X.Vl53l0xAccuracyMode.LONG_RANGE)

    def __del__(self):
        self.tof.stop_ranging()
        self.tof.close()


    def timer_callback(self):
        distance = self.tof.get_distance()

        msg = Range()
        msg.min_range = 0.02
        msg.max_range = 10
        msg.header = Header()
        msg.header.stamp  = time.time()

        msg.range = distance

        self.publisher_.publish(msg)

        




def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
