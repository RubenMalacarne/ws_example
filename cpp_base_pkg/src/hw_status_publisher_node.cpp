#include "rclcpp/rclcpp.hpp"
#include "interfaces_pkg/msg/hardware_status.hpp"


class HardwareStatusPublisherNode : public rclcpp::Node // MODIFY NAME
{

private:
    rclcpp::Publisher<interfaces_pkg::msg::HardwareStatus>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void publishHardwareStatus()
    {
        auto msg = interfaces_pkg::msg::HardwareStatus();
        msg.temperature = 57;
        msg.are_motor_ready = false;
        msg.debug_message = "Motors are too hot!";
        pub_->publish(msg);

    }

public:
    HardwareStatusPublisherNode() : Node("hardware_status_publisher_node") // MODIFY NAME
    {
            pub_ = this->create_publisher<interfaces_pkg::msg::HardwareStatus>("hardware_status_node",10);
            
            timer_ = this-> create_wall_timer(
                        std::chrono::seconds(1),
                        std::bind(&HardwareStatusPublisherNode::publishHardwareStatus,this));
            RCLCPP_INFO(this->get_logger(), "Hardware status publisher has been started");
    }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HardwareStatusPublisherNode>(); // MODIFY NAME
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
