#include "gnss_serial/gnss_serial.hpp"

#include <fcntl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <cstring>
#include <ctime>
#include <limits>
#include <memory>
#include <mutex>
#include <vector>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/int32.hpp"

#define serial_port "/dev/stm32"
#define BAUDRATE B1000000

namespace gnss_serial {

GNSS_Serial::GNSS_Serial()
    : rclcpp::Node("gnss_serial_node"),
      stop_thread(false),
      max_reconnection_attempts_(5),
      reconnection_delay_ms_(1000),
      latitude_(0.0),
      longitude_(0.0),
      altitude_(0.0),
      velocity_(0.0),
      fix_type_(0),
      carrier_solution_(0),
      gnss_valid_(false),
      second_(0),
      gnss_timestamp_valid_(false) {
  // Declare parameters
  this->declare_parameter("portname", std::string("/dev/ttyUSB1"));
  this->declare_parameter("baudrate", 115200);
  this->declare_parameter("publish_rate_hz", 10.0);

  // Get parameters
  portname_ = this->get_parameter("portname").as_string();
  baudrate_ = this->get_parameter("baudrate").as_int();
  double publish_rate = this->get_parameter("publish_rate_hz").as_double();

  // Initialize serial port
  serial_port_ = std::make_unique<serialib>();

  // Create publishers
  gnss_publisher_ =
      this->create_publisher<sensor_msgs::msg::NavSatFix>("gnss/fix", 10);
  velocity_publisher_ =
      this->create_publisher<geometry_msgs::msg::TwistStamped>(
          "gnss/fix_velocity", 10);
  fix_type_publisher_ =
      this->create_publisher<std_msgs::msg::Int32>("gnss/fix_type", 10);

  // Initialize serial connection
  if (!initialize_serial_connection()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize serial connection");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "GNSS Hardware node initialized");
}
GNSS_Serial::~GNSS_Serial() {
  stop_thread = true;
  if (serial_port_) {
    serial_port_->closeDevice();
  }
}

bool GNSS_Serial::initialize_serial_connection() {
  if (!serial_port_ ||
      serial_port_->openDevice(portname_.c_str(), baudrate_) != 1) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s",
                 portname_.c_str());
  }
  RCLCPP_INFO(this->get_logger(), "Successfully connected to serial port: %s",
              portname_.c_str());
  return true;
}

void GNSS_Serial::read() {
  char buffer[256] = {0};  // Increased buffer size

  // Check if device is still connected before attempting to read
  if (!serial_port_ || !serial_port_->isDeviceOpen()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Serial device not open, attempting reconnection...");
    return;
  }

  bytesRead = serial_port_->readString(buffer, '\n', sizeof(buffer) - 1, 10000);

  if (bytesRead > 0) {
    parse_gnss_data(buffer);

    if (gnss_valid_) {
      rclcpp::Time timestamp;
      if (gnss_timestamp_valid_) {
        auto now = std::chrono::system_clock::now();
        auto current_time = std::chrono::system_clock::to_time_t(now);
        std::tm utc_tm{};
        gmtime_r(&current_time, &utc_tm);
        utc_tm.tm_hour = static_cast<int>(hour_);
        utc_tm.tm_min = static_cast<int>(minute_);
        utc_tm.tm_sec = static_cast<int>(second_);

        std::time_t gnss_time = timegm(&utc_tm);
        timestamp =
            rclcpp::Time(static_cast<int64_t>(gnss_time), 0, RCL_SYSTEM_TIME);
      } else {
        timestamp = this->get_clock()->now();
      }

      auto gnss_msg = sensor_msgs::msg::NavSatFix();
      gnss_msg.header.stamp = timestamp;
      gnss_msg.header.frame_id = "gnss_link";

      gnss_msg.latitude = latitude_;
      gnss_msg.longitude = longitude_;
      gnss_msg.altitude = altitude_;

      // Set status based on fix type
      if (fix_type_ >= 3) {
        gnss_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
      } else if (fix_type_ == 2) {
        gnss_msg.status.status =
            sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX;
      } else {
        gnss_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
      }

      gnss_msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

      // Set covariance (simplified - you may want to get actual values from
      // GNSS)
      double position_covariance = 1.0;     // 1 meter standard deviation
      if (carrier_solution_ == 2) {         // RTK Fix
        position_covariance = 0.01;         // 1 cm
      } else if (carrier_solution_ == 1) {  // RTK Float
        position_covariance = 0.1;          // 10 cm
      }

      gnss_msg.position_covariance[0] = position_covariance;  // East
      gnss_msg.position_covariance[4] = position_covariance;  // North
      gnss_msg.position_covariance[8] =
          position_covariance * 2;  // Up (usually less accurate)
      gnss_msg.position_covariance_type =
          sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

      gnss_publisher_->publish(gnss_msg);

      // Create and publish velocity message
      auto velocity_msg = geometry_msgs::msg::TwistStamped();
      velocity_msg.header.stamp = timestamp;
      velocity_msg.header.frame_id = "gnss_link";
      velocity_msg.twist.linear.x = velocity_;  // Forward velocity
      velocity_msg.twist.linear.y = 0.0;
      velocity_msg.twist.linear.z = 0.0;
      velocity_msg.twist.angular.x = 0.0;
      velocity_msg.twist.angular.y = 0.0;
      velocity_msg.twist.angular.z = 0.0;

      velocity_publisher_->publish(velocity_msg);

      // Create and publish fix type message
      auto fix_type_msg = std_msgs::msg::Int32();
      fix_type_msg.data = fix_type_;
      fix_type_publisher_->publish(fix_type_msg);
    }
  }
}

void GNSS_Serial::parse_gnss_data(const char* raw_data) {
  // Expected format: "lat,lon,alt,vel,fix,rtk,hour,min,sec"

  double lat, lon, alt, vel;
  int fix, rtk;
  int hour_tmp = 0;
  int minute_tmp = 0;
  int second_tmp = 0;

  // Try to parse the custom format - handle concatenated e and f fields
  int parsed =
      sscanf(raw_data, "%lf,%lf,%lf,%lf,%d,%d,%d,%d,%d", &lat, &lon, &alt, &vel,
             &fix, &rtk, &hour_tmp, &minute_tmp, &second_tmp);

  if (parsed >= 6) {
    latitude_ = lat;
    longitude_ = lon;
    altitude_ = alt;
    velocity_ = vel;
    fix_type_ = fix;
    carrier_solution_ = rtk;

    if (parsed >= 9) {
      bool time_fields_valid = hour_tmp >= 0 && hour_tmp < 24 &&
                               minute_tmp >= 0 && minute_tmp < 60 &&
                               second_tmp >= 0 && second_tmp < 60;

      if (time_fields_valid) {
        hour_ = static_cast<uint8_t>(hour_tmp);
        minute_ = static_cast<uint8_t>(minute_tmp);
        second_ = static_cast<uint8_t>(second_tmp);
        gnss_timestamp_valid_ = true;
      } else {
        gnss_timestamp_valid_ = false;
        RCLCPP_WARN(this->get_logger(),
                    "Received GNSS time with invalid fields: hour=%d, "
                    "minute=%d, second=%d",
                    hour_tmp, minute_tmp, second_tmp);
      }
    } else {
      gnss_timestamp_valid_ = false;
    }

    gnss_valid_ = true;
    // RCLCPP_WARN(this->get_logger(),
    //             "Successfully parsed GNSS data: lat=%.6f, lon=%.6f, alt=%.2f,
    //             " "vel=%.2f, fix=%d, rtk=%d", lat, lon, alt, vel, fix, rtk);
  } else {
    gnss_valid_ = false;
    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Failed to parse GNSS data: '%s', parsed %d of 9 fields", raw_data,
        parsed);
  }
}

}  // namespace gnss_serial

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<gnss_serial::GNSS_Serial>();

  rclcpp::executors::SingleThreadedExecutor exec;

  rclcpp::Duration timer_duration = rclcpp::Duration::from_seconds(1.0 / 10.0);
  rclcpp::TimerBase::SharedPtr timer =
      rclcpp::create_timer(node, node->get_clock(), timer_duration,
                           [node]() -> void { node->read(); });
  exec.add_node(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
