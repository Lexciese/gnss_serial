#ifndef GNSS_SERIAL__GNSS_SERIAL_HPP_
#define GNSS_SERIAL__GNSS_SERIAL_HPP_

#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/int32.hpp"
#include "serialib.hpp"

using namespace std;

namespace gnss_serial {

  class GNSS_Serial : public rclcpp::Node {
  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(GNSS_Serial);

    GNSS_Serial();
    ~GNSS_Serial();

    void parse_gnss_data(const char* raw_data);
    void read();
    bool initialize_serial_connection();

  private:
    std::unique_ptr<serialib> serial_port_;
    std::atomic<bool> stop_thread;

    std::string portname_;
    int baudrate_;

    // GNSS data
    uint8_t hour_;
    uint8_t minute_;
    uint8_t second_;
    double latitude_;
    double longitude_;
    double altitude_;
    double velocity_;
    int fix_type_;
    int carrier_solution_;
    bool gnss_valid_;
    bool gnss_timestamp_valid_;
    int bytesRead; 

    // USB reconnection parameters
    int max_reconnection_attempts_;
    int reconnection_delay_ms_;

    // ROS2 Publishers
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr fix_type_publisher_;
    
    // Timer for periodic reading
    rclcpp::TimerBase::SharedPtr timer_;

    // Attempts to reconnect to the USB device
    bool attempt_reconnection();
  };

}  // namespace gnss_serial

#endif  // GNSS_SERIAL__GNSS_SERIAL_HPP_
