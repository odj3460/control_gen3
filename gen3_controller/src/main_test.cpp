#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace std::chrono_literals;

class PositionControllerPublisher : public rclcpp::Node
{
public:
    PositionControllerPublisher()
    : Node("position_controller_publisher"), count_(0)
    {
        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/position_controller/commands", 10);
        initialize_message();
        timer_ = this->create_wall_timer(
            2ms, std::bind(&PositionControllerPublisher::publish_message, this));
    }

private:
    void initialize_message()
    {
        // Setting up the layout of the message only once
        message_.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
        message_.layout.dim[0].label = "";
        message_.layout.dim[0].size = 7;
        message_.layout.dim[0].stride = 7;
        message_.layout.data_offset = 0;
    }

    void publish_message()
    {
        // Setting the data for the message
        message_.data = {-0.0000001, 0.34934, -3.14156, -1.74539, 0.0, -1.04725, 1.57034};

        // Publishing the message
        publisher_->publish(message_);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message_.layout.dim[0].label.c_str());
        count_++;
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    std_msgs::msg::Float64MultiArray message_; // Moved message object here
    size_t count_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PositionControllerPublisher>());
    rclcpp::shutdown();
    
    return 0;
}
