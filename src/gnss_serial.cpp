#include "gnss_serial/gnss_serial.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <sys/select.h>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/int32.hpp"

#define serial_port "/dev/stm32"
#define BAUDRATE B1000000

namespace gnss_serial
{

  GNSS_Serial::GNSS_Serial()
    : rclcpp::Node("gnss_serial_node"), stop_thread(false), max_reconnection_attempts_(5), 
      reconnection_delay_ms_(1000), latitude_(0.0), longitude_(0.0), altitude_(0.0), velocity_(0.0),
      fix_type_(0), carrier_solution_(0), gnss_valid_(false), gnss_seconds_(0), gnss_timestamp_valid_(false)
  {
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
    gnss_publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("gnss/fix", 10);
    velocity_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("gnss/fix_velocity", 10);
    fix_type_publisher_ = this->create_publisher<std_msgs::msg::Int32>("gnss/fix_type", 10);

    // Initialize serial connection
    if (!initialize_serial_connection()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize serial connection");
      return;
    }

    // Create timer for periodic reading based on publish rate
    auto timer_period = std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate));
    timer_ = this->create_wall_timer(timer_period, std::bind(&GNSS_Serial::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "GNSS Hardware node initialized");
  }
  GNSS_Serial::~GNSS_Serial()
  {
    stop_thread = true;
    if (serial_port_) {
      serial_port_->closeDevice();
    }
  }

  bool GNSS_Serial::initialize_serial_connection()
  {
    if (!serial_port_ || serial_port_->openDevice(portname_.c_str(), baudrate_) != 1) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", portname_.c_str());
      if (!attempt_reconnection()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to reconnect during initialization");
        return false;
      }
    }
    RCLCPP_INFO(this->get_logger(), "Successfully connected to serial port: %s", portname_.c_str());
    return true;
  }

  void GNSS_Serial::timer_callback()
  {
    // rclcpp::spin_some(this->get_node_base_interface());

    char buffer[256] = { 0 };  // Increased buffer size
    
    // Check if device is still connected before attempting to read
    if (!serial_port_ || !serial_port_->isDeviceOpen()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                           "Serial device not open, attempting reconnection...");
      if (!attempt_reconnection()) {
        return;
      }
    }
    
    // Try reading with different terminators with longer timeout for 5Hz data
    bytesRead = serial_port_->readString(buffer, '\n', sizeof(buffer) - 1, 250);
    
    if (bytesRead > 0) {
      // Remove any trailing whitespace or control characters
      // while (bytesRead > 0 && (buffer[bytesRead-1] == '\n' || buffer[bytesRead-1] == '\r' || buffer[bytesRead-1] == ' ')) {
      //   buffer[--bytesRead] = '\0';
      // }
      
      // RCLCPP_DEBUG(this->get_logger(), "Raw data received (%d bytes): %s", bytesRead, buffer);
      
      parse_gnss_data(buffer);
      
      if (gnss_valid_) {
        // Create and publish GNSS NavSatFix message

        rclcpp::Time timestamp;
        if (gnss_timestamp_valid_) {
          // Convert GNSS seconds to ROS time
          // Assuming gnss_seconds is seconds since some epoch (adjust as needed)
          timestamp = rclcpp::Time(gnss_seconds_, 0);
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
          gnss_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX;
        } else {
          gnss_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
        }
        
        gnss_msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
        
        // Set covariance (simplified - you may want to get actual values from GNSS)
        double position_covariance = 1.0; // 1 meter standard deviation
        if (carrier_solution_ == 2) { // RTK Fix
          position_covariance = 0.01; // 1 cm
        } else if (carrier_solution_ == 1) { // RTK Float
          position_covariance = 0.1; // 10 cm
        }
        
        gnss_msg.position_covariance[0] = position_covariance; // East
        gnss_msg.position_covariance[4] = position_covariance; // North
        gnss_msg.position_covariance[8] = position_covariance * 2; // Up (usually less accurate)
        gnss_msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
        
        gnss_publisher_->publish(gnss_msg);
        
        // Create and publish velocity message
        auto velocity_msg = geometry_msgs::msg::TwistStamped();
        velocity_msg.header.stamp = this->get_clock()->now();
        velocity_msg.header.frame_id = "gnss_link";
        velocity_msg.twist.linear.x = velocity_; // Forward velocity
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
      
    } else {
      // Only trigger reconnection if we consistently get no data
      // static int no_data_count = 0;
      // no_data_count++;
      
      // if (no_data_count >= 10) { // 10 consecutive failures (2 seconds at 5Hz)
      //   RCLCPP_ERROR(this->get_logger(), "No data received for 2 seconds. Attempting to reconnect...");
      //   no_data_count = 0;
      //   if (!attempt_reconnection()) {
      //     RCLCPP_ERROR(this->get_logger(), "Reconnection attempts failed");
      //   }
      // } else {
      //   RCLCPP_DEBUG(this->get_logger(), "No data received (attempt %d/10)", no_data_count);
      // }
    }
  }

  void GNSS_Serial::parse_gnss_data(const char* raw_data) {
    // Expected format: "a:-7.767013b:110.376342c:165.60d:0.07e:3f:0"
    
    double lat, lon, alt, vel;
    int fix, rtk;
    int gnss_seconds = 0;
    
    // Try to parse the custom format - handle concatenated e and f fields
    int parsed = sscanf(raw_data, "a:%lf,b:%lf,c:%lf,d:%lf,e:%d,f:%d,g:%d",
               &lat, &lon, &alt, &vel, &fix, &rtk, &gnss_seconds);
    
    if (parsed >= 6) {
      latitude_ = lat;
      longitude_ = lon;
      altitude_ = alt;
      velocity_ = vel;
      fix_type_ = fix;
      carrier_solution_ = rtk;

      if (parsed == 7) {
        gnss_seconds_ = gnss_seconds;
        gnss_timestamp_valid_ = true;
      } else {
        gnss_timestamp_valid_ = false;
      }
    
      gnss_valid_ = true;
      RCLCPP_WARN(this->get_logger(), 
                  "Successfully parsed GNSS data: lat=%.6f, lon=%.6f, alt=%.2f, vel=%.2f, fix=%d, rtk=%d", 
                  lat, lon, alt, vel, fix, rtk);
    } else {
      gnss_valid_ = false;
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                          "Failed to parse GNSS data: '%s', parsed %d of 6 fields", raw_data, parsed);
    }
  }

  bool GNSS_Serial::attempt_reconnection() {
    RCLCPP_INFO(this->get_logger(), "Attempting to reconnect to USB device...");
    
    // Close the device first if it's open
    if (serial_port_) {
      serial_port_->closeDevice();
    }
    
    // Wait a bit before trying to reconnect
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    // Try to reconnect multiple times
    for (int attempt = 1; attempt <= max_reconnection_attempts_; ++attempt) {
      RCLCPP_INFO(this->get_logger(), 
                 "Reconnection attempt %d of %d", attempt, max_reconnection_attempts_);
      
      if (serial_port_->openDevice(portname_.c_str(), baudrate_) == 1) {
        RCLCPP_INFO(this->get_logger(), "Successfully reconnected to USB device");
        
        // Clear any pending data after reconnection
        char flush_buffer[256];
        serial_port_->readBytes(flush_buffer, sizeof(flush_buffer), 100);
        
        return true;
      }
      
      // Wait before trying again
      std::this_thread::sleep_for(std::chrono::milliseconds(reconnection_delay_ms_));
    }
    
    RCLCPP_ERROR(this->get_logger(), 
                "Failed to reconnect after %d attempts", max_reconnection_attempts_);
    return false;
  }

}  // namespace gnss_serial

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<gnss_serial::GNSS_Serial>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
