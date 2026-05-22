#include "my_franka_controllers/raw_usb_joy_node.hpp"
#include <stdexcept>

RawUsbJoyNode::RawUsbJoyNode()
    : Node("joy_node")
{
    joy_pub = this->create_publisher<sensor_msgs::msg::Joy>("/joy", 10);

    if (libusb_init(&ctx) < 0) {
        RCLCPP_FATAL(this->get_logger(), "Failed to initialize libusb");
        throw std::runtime_error("Libusb init failed");
    }

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

    timer = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&RawUsbJoyNode::poll_usb, this)
    );
    
    RCLCPP_INFO(this->get_logger(), "Direct USB Endpoint Reader Started Successfully.");
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

    int r = libusb_interrupt_transfer(dev_handle, 0x81, buffer, sizeof(buffer), &transferred, 0);
    
    if (r == 0 && transferred >= 20) {
        auto joy_msg = sensor_msgs::msg::Joy();
        joy_msg.header.stamp = this->get_clock()->now();
        joy_msg.header.frame_id = "joy_link";
        
        joy_msg.axes.resize(4, 0.0);
        joy_msg.buttons.resize(8, 0);

        // Buttons
        uint8_t button_byte = buffer[3];
        joy_msg.buttons[4] = (button_byte & 0x01) ? 1 : 0; // LB
        joy_msg.buttons[5] = (button_byte & 0x02) ? 1 : 0; // RB

        // Triggers (LT & RT are 0-255 values on bytes 4 and 5)
        joy_msg.buttons[6] = (buffer[4] > 100) ? 1 : 0; // LT
        joy_msg.buttons[7] = (buffer[5] > 100) ? 1 : 0; // RT

        // Axes (16-bit little endian)
        int16_t lx = (int16_t)((buffer[7] << 8) | buffer[6]);
        int16_t ly = (int16_t)((buffer[9] << 8) | buffer[8]);
        int16_t rx = (int16_t)((buffer[11] << 8) | buffer[10]);
        int16_t ry = (int16_t)((buffer[13] << 8) | buffer[12]);

        joy_msg.axes[0] = (float)lx / 32768.0f; // Left Stick X
        joy_msg.axes[1] = (float)ly / 32768.0f; // Left Stick Y
        joy_msg.axes[2] = (float)rx / 32768.0f; // Right Stick X
        joy_msg.axes[3] = (float)ry / 32768.0f; // Right Stick Y

        joy_pub->publish(joy_msg);
    }
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RawUsbJoyNode>());
    rclcpp::shutdown();
    return 0;
}