#ifndef RAW_USB_JOY_NODE_HPP_
#define RAW_USB_JOY_NODE_HPP_

#include <libusb-1.0/libusb.h>
#include <cstdint>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

class RawUsbJoyNode : public rclcpp::Node {
public:
    RawUsbJoyNode();
    ~RawUsbJoyNode() override;

private:
    void poll_usb();

    libusb_context *ctx{nullptr};
    libusb_device_handle *dev_handle{nullptr};
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_pub;
    rclcpp::TimerBase::SharedPtr timer;
};

#endif  // RAW_USB_JOY_NODE_HPP_