import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Header
import serial
import threading
import sys
import select
import tty
import termios

try:
    ser = serial.Serial('/dev/ttyUSB1', baudrate=115200, timeout=1)
except serial.SerialException as e:
    print(e)

class Panel(Node):

    def __init__(self):
        super().__init__('panel')
        self.publisher_rover = self.create_publisher(NavSatFix, 'lora/ublox_rover/fix', 10)
        self.publisher_base = self.create_publisher(NavSatFix, 'lora/ublox_moving_base/fix', 10)
        ser.reset_input_buffer()
        self.sending_allowed = True
        self.stop_event = threading.Event()
        self.key_thread = threading.Thread(target=self.check_for_keypress)
        self.key_thread.start()

        # Use a timer with a short period to frequently check for serial input
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').strip()
            self.get_logger().info(f'Received: {line}')
            if line == "sending stopped":
                self.sending_allowed = False
                self.get_logger().info(f'Response: {line}')
            else:
                try:
                    dane = line.split(',')
                    topic, latitude, longitude = dane
                    lora_lat = float(latitude)
                    lora_long = float(longitude)
                    self.get_logger().info(f'{lora_lat},{lora_long}')
                    self.publish_data_to_ros(topic, lora_lat, lora_long)
                except ValueError:
                    self.get_logger().error('Invalid data received.')

    def publish_data_to_ros(self, topic, lora_latitude, lora_longitude):
        msg = NavSatFix()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gps_frame'
        msg.status = NavSatStatus()
        msg.status.status = NavSatStatus.STATUS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS
        msg.latitude = lora_latitude
        msg.longitude = lora_longitude
        msg.altitude = 0.0
        msg.position_covariance = [0.0] * 9
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        if topic == "b":
            self.publisher_base.publish(msg)
        elif topic == "r":
            self.publisher_rover.publish(msg)
        else:
            self.get_logger().info('Wrong topic sent!')

    def check_for_keypress(self):
        while not self.stop_event.is_set():
            key = get_key()
            if key == ' ':
                self.send_request()

    def send_request(self):
        if self.sending_allowed:
            self.get_logger().info('Sending request to trigger service')
            ser.write(b'Trigger request\n')

    def destroy(self):
        self.stop_event.set()
        self.key_thread.join()
        self.destroy_node()

def get_key():
    old_settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
    return key

def main(args=None):
    rclpy.init(args=args)
    panel = Panel()

    try:
        while rclpy.ok():
            rclpy.spin_once(panel, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass

    panel.destroy()
    rclpy.shutdown()
    ser.close()

if __name__ == '__main__':
    main()
