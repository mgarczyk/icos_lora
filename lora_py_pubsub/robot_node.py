import rclpy
import serial
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

try:
    ser=serial.Serial('/dev/ttyUSB0',baudrate=115200,timeout=1)

except serial.SerialException as e:
    print(e)

class Robot(Node):
    def __init__(self):
        super().__init__('robot')
        self.subscription_rover = self.create_subscription(NavSatFix,'/ublox_rover/fix',self.listener_callback_rover,10)
        self.subscription_base = self.create_subscription(NavSatFix,'/ublox_moving_base/fix',self.listener_callback_base,10)

    def listener_callback_rover(self, msg):
        data=f"r,{msg.latitude},{msg.longitude}"
        ser.write(data.encode('utf-8'))
        self.get_logger().info(f'data {msg.latitude},{msg.longitude}.\n')

    def listener_callback_base(self, msg):
        data=f"b,{msg.latitude},{msg.longitude}\n"
        ser.write(data.encode('utf-8'))
        self.get_logger().info(f'data {msg.latitude},{msg.longitude}.')

def main(args=None):
    try:
        rclpy.init(args=args)
        robot = Robot()
        rclpy.spin(robot)
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        robot.destroy_node()
        rclpy.shutdown()

    except KeyboardInterrupt:
        robot.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()