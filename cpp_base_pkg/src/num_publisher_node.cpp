#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"


class NumberPublisherNode : public rclcpp::Node // MODIFY NAME
{
private:
    
    int number_;
    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr number_publisher_;
    rclcpp::TimerBase::SharedPtr number_timer_;
    
    void publishNumber()
    {
        auto msg = example_interfaces::msg::Int64();
        msg.data = number_;
        number_publisher_ -> publish(msg);

        
    } 

public:
    NumberPublisherNode() : Node("number_publisher_node"), number_(2) // MODIFY NAME
    {
        number_publisher_ = this-> create_publisher<example_interfaces::msg::Int64>("number", 10);
        number_timer_= this->create_wall_timer(std::chrono::seconds(1),
        std::bind(&NumberPublisherNode::publishNumber,this));

        RCLCPP_INFO(this->get_logger(), "number publisher has been started. ");
        
    }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberPublisherNode>(); // MODIFY NAME
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
