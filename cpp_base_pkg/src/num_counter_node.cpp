#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"

class NumberCounterNode : public rclcpp::Node // MODIFY NAME
{
private:

    int counter_;
    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr counter_publisher_;
    rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr number_subscriber_;

    void callbackNumber(const example_interfaces::msg::Int64::SharedPtr msg )
    {
        counter_ += msg->data;
        auto newMsg = example_interfaces::msg::Int64();
        newMsg.data = counter_;
        counter_publisher_ -> publish(newMsg);
    }

public:
    NumberCounterNode() : Node("number_counter_node"), counter_(0) // MODIFY NAME
    {
        counter_publisher_ = this-> create_publisher<example_interfaces::msg::Int64>("number_count", 10);
        number_subscriber_ = this-> create_subscription<example_interfaces::msg::Int64>(
            "number", 10, std::bind(&NumberCounterNode::callbackNumber,this, std::placeholders::_1));
    }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberCounterNode>(); // MODIFY NAME
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
