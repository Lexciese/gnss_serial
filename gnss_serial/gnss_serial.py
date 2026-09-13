import datetime
import math
import sys
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.time import Time
from sensor_msgs.msg import NavSatFix, NavSatStatus
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Int32
import serial


class GNSSSerial(Node):
    def __init__(self) -> None:
        super().__init__('gnss_serial_node')

        # Declare parameters
        self.declare_parameter('portname', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('publish_rate_hz', 10.0)

        # Get parameters
        self.portname: str = self.get_parameter('portname').get_parameter_value().string_value
        self.baudrate: int = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.publish_rate: float = self.get_parameter('publish_rate_hz').get_parameter_value().double_value

        # Member variables state
        self.latitude: float = 0.0
        self.longitude: float = 0.0
        self.altitude: float = 0.0
        self.velocity: float = 0.0
        self.fix_type: int = 0
        self.carrier_solution: int = 0
        self.gnss_valid: bool = False

        self.hour: int = 0
        self.minute: int = 0
        self.second: int = 0
        self.nanosecond: int = 0
        self.gnss_timestamp_valid: bool = False

        self.serial_port: Optional[serial.Serial] = None

        # Custom QoS setup (matching C++ Best Effort QoS)
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        # Create publishers
        self.gnss_publisher_ = self.create_publisher(NavSatFix, 'gnss/fix', qos)
        self.velocity_publisher_ = self.create_publisher(TwistStamped, 'gnss/fix_velocity', qos)
        self.fix_type_publisher_ = self.create_publisher(Int32, 'gnss/fix_type', qos)

        self.get_logger().info('GNSS Hardware node initialized')

    def on_init(self) -> bool:
        try:
            self.serial_port = serial.Serial(
                port=self.portname,
                baudrate=self.baudrate,
                timeout=0.005  # non-blocking / quick read timeout (5ms)
            )

            # DTR Toggle
            self.serial_port.dtr = False
            self.serial_port.dtr = True

            time.sleep(1.0)
            self.serial_port.reset_input_buffer()

            self.get_logger().info(f'Successfully connected to serial port: {self.portname}')
            return True
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port {self.portname}: {e}')
            return False

    def read(self) -> None:
        if self.serial_port is None or not self.serial_port.is_open:
            self.get_logger().warn(
                'Serial device not open, attempting reconnection...',
                throttle_duration_sec=5.0
            )
            return

        latest_line = None
        has_new_data = False

        try:
            # Read all lines available in the input buffer to fetch only the newest line
            while self.serial_port.in_waiting > 0:
                line = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    latest_line = line
                    has_new_data = True
        except serial.SerialException as e:
            self.get_logger().error(f'Error reading from serial port: {e}')
            return

        if has_new_data and latest_line:
            self.parse(latest_line)

            if self.gnss_valid:
                timestamp: Time
                if self.gnss_timestamp_valid:
                    now_utc = datetime.datetime.now(datetime.timezone.utc)
                    current_date = now_utc.date()

                    # UTC Midnight Rollover Protection
                    if now_utc.hour == 23 and self.hour == 0:
                        current_date += datetime.timedelta(days=1)
                    elif now_utc.hour == 0 and self.hour == 23:
                        current_date -= datetime.timedelta(days=1)

                    gnss_dt = datetime.datetime(
                        year=current_date.year,
                        month=current_date.month,
                        day=current_date.day,
                        hour=self.hour,
                        minute=self.minute,
                        second=self.second,
                        tzinfo=datetime.timezone.utc
                    )

                    seconds_since_epoch = int(gnss_dt.timestamp())
                    timestamp = Time(seconds=seconds_since_epoch, nanoseconds=self.nanosecond)
                else:
                    timestamp = self.get_clock().now()

                # 1. NavSatFix Message
                gnss_msg = NavSatFix()
                gnss_msg.header.stamp = timestamp.to_msg()
                gnss_msg.header.frame_id = 'gnss_link'

                gnss_msg.latitude = self.latitude
                gnss_msg.longitude = self.longitude
                gnss_msg.altitude = self.altitude

                # Status based on fix type
                if self.fix_type >= 3:
                    gnss_msg.status.status = NavSatStatus.STATUS_FIX
                elif self.fix_type == 2:
                    gnss_msg.status.status = NavSatStatus.STATUS_SBAS_FIX
                else:
                    gnss_msg.status.status = NavSatStatus.STATUS_NO_FIX

                gnss_msg.status.service = NavSatStatus.SERVICE_GPS

                # Position covariance based on RTK status
                position_covariance = 1.0  # 1 meter default std dev
                if self.carrier_solution == 2:      # RTK Fix
                    position_covariance = 0.01      # 1cm
                elif self.carrier_solution == 1:    # RTK Float
                    position_covariance = 0.02      # 2cm
                elif self.carrier_solution == 0:
                    position_covariance = 0.2       # 20cm

                gnss_msg.position_covariance[0] = position_covariance      # East
                gnss_msg.position_covariance[4] = position_covariance      # North
                gnss_msg.position_covariance[8] = position_covariance * 2  # Up (less accurate)
                gnss_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

                self.gnss_publisher_.publish(gnss_msg)

                # 2. TwistStamped Message
                velocity_msg = TwistStamped()
                velocity_msg.header.stamp = timestamp.to_msg()
                velocity_msg.header.frame_id = 'gnss_link'
                velocity_msg.twist.linear.x = self.velocity
                velocity_msg.twist.linear.y = 0.0
                velocity_msg.twist.linear.z = 0.0
                velocity_msg.twist.angular.x = 0.0
                velocity_msg.twist.angular.y = 0.0
                velocity_msg.twist.angular.z = 0.0

                self.velocity_publisher_.publish(velocity_msg)

                # 3. Fix Type Message
                fix_type_msg = Int32()
                fix_type_msg.data = self.fix_type
                self.fix_type_publisher_.publish(fix_type_msg)

    def parse(self, raw_data: str) -> None:
        tokens = raw_data.split(',')
        if len(tokens) >= 6:
            try:
                self.latitude = float(tokens[0])
                self.longitude = float(tokens[1])
                self.altitude = float(tokens[2])
                self.velocity = float(tokens[3])
                self.fix_type = int(tokens[4])
                self.carrier_solution = int(tokens[5])

                if len(tokens) >= 10:
                    hour_tmp = int(tokens[6])
                    minute_tmp = int(tokens[7])
                    second_tmp = int(tokens[8])
                    nanosecond_tmp = int(tokens[9])

                    time_fields_valid = (
                        0 <= hour_tmp < 24 and
                        0 <= minute_tmp < 60 and
                        0 <= second_tmp < 60
                    )

                    if time_fields_valid:
                        self.hour = hour_tmp
                        self.minute = minute_tmp
                        self.second = second_tmp
                        self.nanosecond = nanosecond_tmp
                        self.gnss_timestamp_valid = True
                    else:
                        self.gnss_timestamp_valid = False
                        self.get_logger().warn(
                            f'Received GNSS time with invalid fields: hour={hour_tmp}, '
                            f'minute={minute_tmp}, second={second_tmp}, nsec={nanosecond_tmp}'
                        )
                else:
                    self.gnss_timestamp_valid = False

                self.gnss_valid = True
            except ValueError:
                self.gnss_valid = False
                self.get_logger().warn(
                    f"Failed to parse numbers from GNSS data: '{raw_data}'",
                    throttle_duration_sec=5.0
                )
        else:
            self.gnss_valid = False
            self.get_logger().warn(
                f"Failed to parse GNSS data: '{raw_data}', parsed {len(tokens)} of expected 9+ fields",
                throttle_duration_sec=5.0
            )

    def destroy_node(self) -> None:
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GNSSSerial()

    if not node.on_init():
        sys.exit(-1)

    # Calculate timer period in seconds
    timer_period = 1.0 / node.publish_rate
    node.create_timer(timer_period, node.read)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
