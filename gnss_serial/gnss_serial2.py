import sys
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
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
        self.declare_parameter('frame_id', 'gnss_link')

        # Get parameters
        self.portname: str = self.get_parameter('portname').get_parameter_value().string_value
        self.baudrate: int = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.publish_rate: float = self.get_parameter('publish_rate_hz').get_parameter_value().double_value
        self.frame_id: str = self.get_parameter('frame_id').get_parameter_value().string_value

        # Committed GNSS state
        self.latitude: float = 0.0
        self.longitude: float = 0.0
        self.altitude: float = 0.0
        self.velocity: float = 0.0
        self.fix_type: int = 0
        self.carrier_solution: int = 0
        self.gnss_valid: bool = False

        # Staging variables for multi-line block assembly
        self._pending_lat: Optional[float] = None
        self._pending_lon: Optional[float] = None
        self._pending_fix: Optional[int] = None
        self._pending_carrier: Optional[int] = None

        # Persistent serial read buffer to prevent mid-line frame tearing
        self._rx_buffer: str = ''
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
                timeout=0.005
            )

            # DTR Toggle
            self.serial_port.dtr = False
            self.serial_port.dtr = True

            time.sleep(1.0)
            self.serial_port.reset_input_buffer()
            self._rx_buffer = ''
            self._reset_pending()

            self.get_logger().info(f'Successfully connected to serial port: {self.portname}')
            return True
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port {self.portname}: {e}')
            return False

    def _reset_pending(self) -> None:
        self._pending_lat = None
        self._pending_lon = None
        self._pending_fix = None
        self._pending_carrier = None

    def read(self) -> None:
        if self.serial_port is None or not self.serial_port.is_open:
            self.get_logger().warn(
                'Serial device not open, attempting reconnection...',
                throttle_duration_sec=5.0
            )
            self.on_init()
            return

        try:
            waiting = self.serial_port.in_waiting
            if waiting > 0:
                chunk = self.serial_port.read(waiting).decode('utf-8', errors='ignore')
                self._rx_buffer += chunk
        except serial.SerialException as e:
            self.get_logger().error(f'Error reading from serial port: {e}')
            try:
                self.serial_port.close()
            except serial.SerialException:
                pass
            return

        # Wait until at least one complete newline-terminated line is available
        if '\n' not in self._rx_buffer:
            return

        # Stamp immediately upon receiving packet data using the ROS 2 host clock
        stamp_msg = self.get_clock().now().to_msg()

        # Split lines and keep any incomplete trailing fragment for the next timer cycle
        lines = self._rx_buffer.split('\n')
        self._rx_buffer = lines[-1]

        # Process all complete lines in order so multi-line blocks assemble cleanly
        new_complete_block = False
        for raw_line in lines[:-1]:
            if self.parse_line(raw_line.strip()):
                new_complete_block = True

        if new_complete_block and self.gnss_valid:
            # 1. NavSatFix Message
            gnss_msg = NavSatFix()
            gnss_msg.header.stamp = stamp_msg
            gnss_msg.header.frame_id = self.frame_id

            gnss_msg.latitude = self.latitude
            gnss_msg.longitude = self.longitude
            gnss_msg.altitude = self.altitude  # Remains 0.0 unless printed by Arduino

            # Status based on fix type
            if self.fix_type >= 3:
                gnss_msg.status.status = NavSatStatus.STATUS_FIX
            elif self.fix_type == 2:
                gnss_msg.status.status = NavSatStatus.STATUS_SBAS_FIX
            else:
                gnss_msg.status.status = NavSatStatus.STATUS_NO_FIX

            gnss_msg.status.service = NavSatStatus.SERVICE_GPS

            # Position covariance (variance in m^2 = std_dev^2)
            std_dev = 1.0  # 1.0 m default
            if self.carrier_solution == 2:      # RTK Fix
                std_dev = 0.01                  # 1 cm
            elif self.carrier_solution == 1:    # RTK Float
                std_dev = 0.05                  # 5 cm
            elif self.carrier_solution == 0 and self.fix_type >= 3:
                std_dev = 0.5                   # 50 cm standalone/SBAS

            variance = std_dev ** 2
            gnss_msg.position_covariance[0] = variance              # East (m^2)
            gnss_msg.position_covariance[4] = variance              # North (m^2)
            gnss_msg.position_covariance[8] = (std_dev * 2.0) ** 2  # Up (m^2)
            gnss_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

            self.gnss_publisher_.publish(gnss_msg)

            # 2. TwistStamped Message (linear.x remains 0.0 unless velocity is printed)
            velocity_msg = TwistStamped()
            velocity_msg.header.stamp = stamp_msg
            velocity_msg.header.frame_id = self.frame_id
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

    def parse_line(self, line: str) -> bool:
        """
        Parses one line from the Arduino multi-line output:
          Latitude: <float>
          Longitude: <float>
          Fix Type: <int>
          RTK Fix | RTK Float | No RTK
        Returns True when a complete 4-field block has been validated and committed.
        """
        if not line:
            return False

        # Ignore startup/status messages from setup() or disconnected loop()
        if line.startswith(('Starting', 'Serial2', 'u-blox', 'Waiting for GNSS')):
            return False

        try:
            if line.startswith('Latitude:'):
                # Start of a new block; clear any partial previous state
                self._reset_pending()
                self._pending_lat = float(line.split(':', 1)[1].strip())

            elif line.startswith('Longitude:'):
                self._pending_lon = float(line.split(':', 1)[1].strip())

            elif line.startswith('Fix Type:'):
                self._pending_fix = int(line.split(':', 1)[1].strip())

            elif line in ('RTK Fix', 'RTK Float', 'No RTK'):
                if line == 'RTK Fix':
                    self._pending_carrier = 2
                elif line == 'RTK Float':
                    self._pending_carrier = 1
                else:
                    self._pending_carrier = 0

                # Carrier solution is the final line of the Arduino block—commit if complete
                if (
                    self._pending_lat is not None
                    and self._pending_lon is not None
                    and self._pending_fix is not None
                    and self._pending_carrier is not None
                ):
                    if not (-90.0 <= self._pending_lat <= 90.0 and -180.0 <= self._pending_lon <= 180.0):
                        self.gnss_valid = False
                        self.get_logger().warn(
                            f'Out-of-range coordinates received: lat={self._pending_lat}, lon={self._pending_lon}',
                            throttle_duration_sec=5.0
                        )
                        self._reset_pending()
                        return False

                    self.latitude = self._pending_lat
                    self.longitude = self._pending_lon
                    self.fix_type = self._pending_fix
                    self.carrier_solution = self._pending_carrier
                    self.gnss_valid = True
                    self._reset_pending()
                    return True

        except ValueError:
            self.gnss_valid = False
            self._reset_pending()
            self.get_logger().warn(
                f"Failed to parse value from GNSS line: '{line}'",
                throttle_duration_sec=5.0
            )

        return False

    def destroy_node(self) -> None:
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GNSSSerial()

    if not node.on_init():
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    timer_period = 1.0 / node.publish_rate
    node.create_timer(timer_period, node.read)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
