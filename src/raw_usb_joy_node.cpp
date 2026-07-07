/// @file raw_usb_joy_node.cpp
/// @brief Direct libusb polling and bitwise input parsing implementation.

#include "lai_franka_controllers/raw_usb_joy_node.hpp"
#include <stdexcept>
#include <algorithm>

namespace lai_franka_controllers {

RawUsbJoyNode::RawUsbJoyNode() : Node("joy_node") {
    joy_pub_ = this->create_publisher<sensor_msgs::msg::Joy>("/joy", 10);

    // Establish native sub-level USB communication system
    if (libusb_init(&ctx_) < 0) {
        RCLCPP_FATAL(this->get_logger(), "Failed to initialize libusb context.");
        throw std::runtime_error("Libusb init failed");
    }

    // VID/PID for Logitech F710/F310 running under X-Input configuration profile
    const uint16_t LOGITECH_VID = 0x046D;
    const uint16_t LOGITECH_PID = 0xC21D;
    
    dev_handle_ = libusb_open_device_with_vid_pid(ctx_, LOGITECH_VID, LOGITECH_PID);
    if (!dev_handle_) {
        RCLCPP_FATAL(this->get_logger(), "Could not find USB Gamepad. Check connection or lsusb IDs.");
        libusb_exit(ctx_);
        throw std::runtime_error("Device not found");
    }

    // Detach system level controller configurations if present
    if (libusb_kernel_driver_active(dev_handle_, 0) == 1) {
        libusb_detach_kernel_driver(dev_handle_, 0);
    }

    if (libusb_claim_interface(dev_handle_, 0) < 0) {
        RCLCPP_FATAL(this->get_logger(), "Cannot claim USB interface endpoint 0.");
        libusb_close(dev_handle_);
        libusb_exit(ctx_);
        throw std::runtime_error("Interface claim failed");
    }

    // 100 Hz scheduling interval updates internal packet array states every 10ms
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&RawUsbJoyNode::poll_usb, this)
    );
    
    RCLCPP_INFO(this->get_logger(), "Direct USB Endpoint Reader Successfully Matched to Teleop Specs.");
}

RawUsbJoyNode::~RawUsbJoyNode() {
    if (dev_handle_) {
        libusb_release_interface(dev_handle_, 0);
        libusb_close(dev_handle_);
    }
    if (ctx_) {
        libusb_exit(ctx_);
    }
}

void RawUsbJoyNode::poll_usb() {
    uint8_t buffer[64];
    int transferred = 0;
    const uint8_t ENDPOINT_IN_ADDR = 0x81;
    const int PACKET_MIN_SIZE = 20;

    // Execute non-blocking interrupt polling check step
    int r = libusb_interrupt_transfer(dev_handle_, ENDPOINT_IN_ADDR, buffer, sizeof(buffer), &transferred, 0);
    
    if (r == 0 && transferred >= PACKET_MIN_SIZE) {
        auto joy_msg = sensor_msgs::msg::Joy();
        joy_msg.header.stamp = this->get_clock()->now();
        joy_msg.header.frame_id = "joy_link";
        
        // Allocate array sizes matching standard Linux gamepad profiles layouts
        const size_t TOTAL_AXES_SIZE = 8;
        const size_t TOTAL_BUTTONS_SIZE = 11;
        joy_msg.axes.resize(TOTAL_AXES_SIZE, 0.0f);
        joy_msg.buttons.resize(TOTAL_BUTTONS_SIZE, 0);

        // Process Digital Action Buttons Array (Byte 3 alignment parsing)
        uint8_t button_byte = buffer[3];
        joy_msg.buttons[0] = (button_byte & 0x10) ? 1 : 0; // Button A
        joy_msg.buttons[1] = (button_byte & 0x20) ? 1 : 0; // Button B
        joy_msg.buttons[2] = (button_byte & 0x40) ? 1 : 0; // Button X
        joy_msg.buttons[3] = (button_byte & 0x80) ? 1 : 0; // Button Y
        joy_msg.buttons[4] = (button_byte & 0x01) ? 1 : 0; // Button LB (Yaw Right)
        joy_msg.buttons[5] = (button_byte & 0x02) ? 1 : 0; // Button RB (Yaw Left)

        // Process Analog Joysticks Sticks (16-Bit Little Endian conversion calculations)
        int16_t lx = (int16_t)((buffer[7]  << 8) | buffer[6]);
        int16_t ly = (int16_t)((buffer[9]  << 8) | buffer[8]);
        int16_t rx = (int16_t)((buffer[11] << 8) | buffer[10]);
        int16_t ry = (int16_t)((buffer[13] << 8) | buffer[12]);

        const float ANALOG_SCALE_FACTOR = 32768.0f;
        joy_msg.axes[0] = static_cast<float>(lx) / ANALOG_SCALE_FACTOR; // Left Stick Left/Right
        joy_msg.axes[1] = static_cast<float>(ly) / ANALOG_SCALE_FACTOR; // Left Stick Up/Down
        joy_msg.axes[3] = static_cast<float>(rx) / ANALOG_SCALE_FACTOR; // Right Stick Left/Right
        joy_msg.axes[4] = static_cast<float>(ry) / ANALOG_SCALE_FACTOR; // Right Stick Up/Down

        // Process Analog Pressure Triggers (Bytes 4 and 5 scaling parsing)
        // Convert raw byte scale [0 to 255] into standard resting profile limits [1.0 to -1.0]
        uint8_t raw_lt = buffer[4];
        uint8_t raw_rt = buffer[5];

        joy_msg.axes[2] = 1.0f - (2.0f * (static_cast<float>(raw_rt) / 255.0f)); // RT Axis Index 2
        joy_msg.axes[5] = 1.0f - (2.0f * (static_cast<float>(raw_lt) / 255.0f)); // LT Axis Index 5

        // Process D-Pad Configurations Array States (Byte 2 isolated lower nibble selection)
        uint8_t dpad_byte = buffer[2] & 0x0F;
        if (dpad_byte == 1) { joy_msg.axes[7] = 1.0f;  } // D-Pad Up
        if (dpad_byte == 5) { joy_msg.axes[7] = -1.0f; } // D-Pad Down
        if (dpad_byte == 3) { joy_msg.axes[6] = -1.0f; } // D-Pad Right
        if (dpad_byte == 7) { joy_msg.axes[6] = 1.0f;  } // D-Pad Left

        joy_pub_->publish(joy_msg);
    }
}

}  // namespace lai_franka_controllers

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<lai_franka_controllers::RawUsbJoyNode>());
    rclcpp::shutdown();
    return 0;
}