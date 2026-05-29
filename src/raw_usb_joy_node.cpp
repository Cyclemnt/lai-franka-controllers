#include "my_franka_controllers/raw_usb_joy_node.hpp"
#include <stdexcept>
#include <algorithm>

RawUsbJoyNode::RawUsbJoyNode()
    : Node("joy_node")
{
    joy_pub = this->create_publisher<sensor_msgs::msg::Joy>("/joy", 10);

    if (libusb_init(&ctx) < 0) {
        RCLCPP_FATAL(this->get_logger(), "Failed to initialize libusb");
        throw std::runtime_error("Libusb init failed");
    }

    // VID/PID for Logitech F710/F310 in X-Input mode
    dev_handle = libusb_open_device_with_vid_pid(ctx, 0x046D, 0xC21D);
    if (!dev_handle) {
        RCLCPP_FATAL(this->get_logger(), "Could not find USB Gamepad. Check lsusb IDs.");
        libusb_exit(ctx);
        throw std::runtime_error("Device not found");
    }

    if (libusb_kernel_driver_active(dev_handle, 0) == 1) {
        libusb_detach_kernel_driver(dev_handle, 0);
    }

    if (libusb_claim_interface(dev_handle, 0) < 0) {
        RCLCPP_FATAL(this->get_logger(), "Cannot claim USB interface 0");
        libusb_close(dev_handle);
        libusb_exit(ctx);
        throw std::runtime_error("Interface claim failed");
    }

    // Increased timer polling slightly to 100Hz (10ms) to ensure smooth input delivery
    timer = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&RawUsbJoyNode::poll_usb, this)
    );
    
    RCLCPP_INFO(this->get_logger(), "Direct USB Endpoint Reader Matched to Teleop Specs.");
}

RawUsbJoyNode::~RawUsbJoyNode() {
    if (dev_handle) {
        libusb_release_interface(dev_handle, 0);
        libusb_close(dev_handle);
    }
    if (ctx) {
        libusb_exit(ctx);
    }
}

void RawUsbJoyNode::poll_usb() {
    uint8_t buffer[64];
    int transferred = 0;

    // Read the raw interrupt endpoint packet
    int r = libusb_interrupt_transfer(dev_handle, 0x81, buffer, sizeof(buffer), &transferred, 0);
    
    if (r == 0 && transferred >= 20) {
        auto joy_msg = sensor_msgs::msg::Joy();
        joy_msg.header.stamp = this->get_clock()->now();
        joy_msg.header.frame_id = "joy_link";
        
        // CRUCIAL FIX: Allocate arrays large enough for Xbox/Logitech standard layouts
        joy_msg.axes.resize(8, 0.0);
        joy_msg.buttons.resize(11, 0);

        // 1. Process Buttons (Byte 3 containing layout actions)
        uint8_t button_byte = buffer[3];
        // Standard mapping array adjustments
        joy_msg.buttons[0] = (button_byte & 0x10) ? 1 : 0; // A Button
        joy_msg.buttons[1] = (button_byte & 0x20) ? 1 : 0; // B Button
        joy_msg.buttons[2] = (button_byte & 0x40) ? 1 : 0; // X Button
        joy_msg.buttons[3] = (button_byte & 0x80) ? 1 : 0; // Y Button
        joy_msg.buttons[4] = (button_byte & 0x01) ? 1 : 0; // LB Button (Triggers Yaw +)
        joy_msg.buttons[5] = (button_byte & 0x02) ? 1 : 0; // RB Button (Triggers Yaw -)

        // 2. Process Analog Joysticks (Signed 16-bit little endian conversions)
        int16_t lx = (int16_t)((buffer[7] << 8) | buffer[6]);
        int16_t ly = (int16_t)((buffer[9] << 8) | buffer[8]);
        int16_t rx = (int16_t)((buffer[11] << 8) | buffer[10]);
        int16_t ry = (int16_t)((buffer[13] << 8) | buffer[12]);

        // Normalize axes exactly to standard range [-1.0, 1.0]
        joy_msg.axes[0] = (float)lx / 32768.0f; // Left Stick Left/Right
        joy_msg.axes[1] = (float)ly / 32768.0f; // Left Stick Up/Down
        joy_msg.axes[3] = (float)rx / 32768.0f; // Right Stick Left/Right
        joy_msg.axes[4] = (float)ry / 32768.0f; // Right Stick Up/Down

        // 3. Process Analog Triggers (LT & RT)
        // Raw bytes give [0 to 255]. Standard Linux joy expects scaling from Rest (1.0) to Fully Pressed (-1.0).
        uint8_t raw_lt = buffer[4];
        uint8_t raw_rt = buffer[5];

        joy_msg.axes[2] = 1.0f - (2.0f * ((float)raw_rt / 255.0f)); // RT Axis mapped to index 2
        joy_msg.axes[5] = 1.0f - (2.0f * ((float)raw_lt / 255.0f)); // LT Axis mapped to index 5

        // 4. Process D-Pad Inputs (Extracted from Byte 2)
        uint8_t dpad_byte = buffer[2] & 0x0F; // Isolate lower nibble
        if (dpad_byte == 1) { joy_msg.axes[7] = 1.0f;  } // D-Pad Up
        if (dpad_byte == 5) { joy_msg.axes[7] = -1.0f; } // D-Pad Down
        if (dpad_byte == 3) { joy_msg.axes[6] = -1.0f; } // D-Pad Right
        if (dpad_byte == 7) { joy_msg.axes[6] = 1.0f;  } // D-Pad Left

        joy_pub->publish(joy_msg);
    }
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RawUsbJoyNode>());
    rclcpp::shutdown();
    return 0;
}