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
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Header
import serial


try:
    ser=serial.Serial('/dev/ttyUSB0',baudrate=115200,timeout=1)
except serial.SerialException as e:
    print(e)

class Panel(Node):

    def __init__(self):
        super().__init__('panel')
        self.publisher_rover = self.create_publisher(NavSatFix, 'lora/ublox_rover/fix', 10)
        self.publisher_base = self.create_publisher(NavSatFix, 'lora/ublox_moving_base/fix', 10)
        ser.reset_input_buffer()
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        

        if ser.in_waiting>0:
            line=ser.readline().decode('utf-8').strip()
            if line:
                try:
                    dane = line.split(',')
                    topic, latitude, longitude = dane
                    lora_lat = float(latitude)
                    lora_long = float(longitude)
                    self.get_logger().info(f'{lora_lat},{lora_long}')
                    self.publish_data_to_ros(topic,lora_lat,lora_long)
                except ValueError:
                    self.get_logger().error('Invalid data recieved.')

    def publish_data_to_ros(self,topic,lora_latitude,lora_longitude):
        msg = NavSatFix()
        msg.header=Header()
        msg.header.stamp=self.get_clock().now().to_msg()
        msg.header.frame_id='gps_frame'
        msg.status=NavSatStatus()
        msg.status.status=NavSatStatus.STATUS_FIX
        msg.status.service=NavSatStatus.SERVICE_GPS
        msg.latitude=lora_latitude
        msg.longitude=lora_longitude
        msg.altitude=0.0
        msg.position_covariance=[0.0]*9
        msg.position_covariance_type=NavSatFix.COVARIANCE_TYPE_UNKNOWN
        if topic == "b":
            self.publisher_base.publish(msg)
        elif topic=="r":
            self.publisher_rover.publish(msg)
        else:
            self.get_logger().info('Wrong topic send!')

   

def main(args=None):
    rclpy.init(args=args)

    panel = Panel()

    rclpy.spin(panel)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    panel.destroy_node()
    rclpy.shutdown()
    ser.close()

if __name__ == '__main__':
    main()
