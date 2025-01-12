#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

using std::placeholders::_1;
using std::placeholders::_2;


class AddTwoIntsServerNode : public rclcpp::Node // MODIFY NAME
{
private:
    
    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr server_;

    void callbackAddTwoInts (const example_interfaces::srv::AddTwoInts::Request::SharedPtr request,
                            const example_interfaces::srv::AddTwoInts::Response::SharedPtr response)
        {
            response->sum = request->a + request->b;
            RCLCPP_INFO(this->get_logger(), "%d + %d = %d", (int)request->a, (int)request->b, (int)response->sum);
        }

public:
    AddTwoIntsServerNode() : Node("add_two_ints_server_node") // MODIFY NAME
    {
        //dentro le parentesi quadre inserire il dataType
        server_ = this->create_service<example_interfaces::srv::AddTwoInts>(
            "add_two_ints",
            std::bind(&AddTwoIntsServerNode::callbackAddTwoInts, this, _1, _2));
        RCLCPP_INFO(this->get_logger(), "Service server has been started.");
    }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AddTwoIntsServerNode>(); // MODIFY NAME
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
