#include "rclcpp/rclcpp.hpp"

class MyNode : public rclcpp::Node // MODIFY NAME
{
private:
    
    rclcpp::TimerBase::SharedPtr timer_;
    int counter_;
    
    void timerCallback()
    {
        counter_++;
        RCLCPP_INFO(this->get_logger(),"hello %d", counter_);
    }
    
public:
    MyNode() : Node("node_name"), counter_(0) // MODIFY NAME
    {
        RCLCPP_INFO(this->get_logger(), "Hello Cpp Node");

        timer_ = this->create_wall_timer(std::chrono::seconds(1),
                                         std::bind(&MyNode::timerCallback, this));
    }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyNode>(); // MODIFY NAME
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
