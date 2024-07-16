import rclpy
import serial
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_srvs.srv import Trigger
import threading

try:
    ser = serial.Serial('/dev/ttyUSB0', baudrate=115200, timeout=1)
except serial.SerialException as e:
    print(e)

class Robot(Node):

    def __init__(self):
        super().__init__('robot')
        self.subscription_rover = self.create_subscription(NavSatFix, '/ublox_rover/fix', self.listener_callback_rover, 10)
        self.subscription_base = self.create_subscription(NavSatFix, '/ublox_moving_base/fix', self.listener_callback_base, 10)
        self.sending_allowed = True  # Flag controlling data sending
        self.rover_data = None
        self.base_data = None
        self.serial_thread = threading.Thread(target=self.check_serial)
        self.serial_thread.start()
        self.publish_timer = self.create_timer(2.0, self.publish_data)  # Timer to publish data every 2 seconds
        self.current_topic = 'rover'  # Start with rover topic

    def listener_callback_rover(self, msg):
        self.rover_data = msg  # Store latest rover data

    def listener_callback_base(self, msg):
        self.base_data = msg  # Store latest base data

    def publish_data(self):
        if self.sending_allowed:
            if self.current_topic == 'rover' and self.rover_data:
                data = f"r,{self.rover_data.latitude},{self.rover_data.longitude}\n"
                ser.write(data.encode('utf-8'))
                self.get_logger().info(f'Rover data sent: {self.rover_data.latitude},{self.rover_data.longitude}')
                self.rover_data = None  # Clear rover data after sending
                self.current_topic = 'base'  # Switch to base topic
            elif self.current_topic == 'base' and self.base_data:
                data = f"b,{self.base_data.latitude},{self.base_data.longitude}\n"
                ser.write(data.encode('utf-8'))
                self.get_logger().info(f'Base data sent: {self.base_data.latitude},{self.base_data.longitude}')
                self.base_data = None  # Clear base data after sending
                self.current_topic = 'rover'  # Switch to rover topic

    def trigger_callback(self, request, response):
        self.sending_allowed = False  # Stop sending data
        response.success = True
        response.message = 'Trigger activated'
        self.get_logger().info('Trigger received')
        data = 'Sending stopped\n'
        ser.write(data.encode('utf-8'))
        return response

    def check_serial(self):
        while rclpy.ok():
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8').strip()
                self.get_logger().info(f'Received: {line}')
                if line == 'Trigger request':
                    self.handle_trigger_request()

    def handle_trigger_request(self):
        self.sending_allowed = False
        data = 'Sending stopped\n'
        ser.write(data.encode('utf-8'))
        self.get_logger().info('Sending stopped due to trigger request')

    def destroy(self):
        self.destroy_node()
        ser.close()

def main(args=None):
    rclpy.init(args=args)
    robot = Robot()

    try:
        rclpy.spin(robot)
    except KeyboardInterrupt:
        pass

    robot.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

