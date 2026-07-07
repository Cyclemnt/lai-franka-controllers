/// @file raw_usb_joy_node.hpp
/// @brief Direct libusb interrupt endpoint polling node for Logitech/Xbox controllers.
/// @author Clement Lamouller
/// @date 2026

#ifndef RAW_USB_JOY_NODE_HPP_
#define RAW_USB_JOY_NODE_HPP_

#include <libusb-1.0/libusb.h>
#include <cstdint>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

namespace lai_franka_controllers {

/// @class RawUsbJoyNode
/// @brief Bypasses the Linux kernel joystick subsystems to read raw controller state via libusb.
///
/// This node claims USB interface endpoints directly and scales data into standard
/// Linux sensor_msgs::msg::Joy array configurations for the teleoperation engine.
class RawUsbJoyNode : public rclcpp::Node {
public:
    /// @brief Constructor for the Raw USB Joystick Node.
    RawUsbJoyNode();
    
    /// @brief Overridden Destructor handling clean USB interface releasing.
    ~RawUsbJoyNode() override;

private:
    /// @brief Reads and parses raw interrupt endpoint data packets from the gamepad controller.
    void poll_usb();

    // ---- Libusb Native Context Handles ----
    libusb_context *ctx_{nullptr};
    libusb_device_handle *dev_handle_{nullptr};
    
    // ---- ROS 2 Communication Interfaces ----
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace lai_franka_controllers

#endif  // RAW_USB_JOY_NODE_HPP_