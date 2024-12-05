#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/spawn.hpp"
#include "turtlesim/srv/kill.hpp"

#include "interfaces_pkg/msg/turtle.hpp"
#include "interfaces_pkg/msg/turtle_array.hpp"
#include "interfaces_pkg/srv/catch_turtle.hpp"

#include <cmath>
#include <vector>
#include <memory>
#include <thread>

class TurtleSpawnNode : public rclcpp::Node
{
private:
    std::string turtle_name_prefix_;
    int turtle_counter_;
    std::vector<std::shared_ptr<std::thread>> spawn_turtle_threads_;
    std::vector<std::shared_ptr<std::thread>> kill_turtle_threads_;
    std::vector<interfaces_pkg::msg::Turtle> alive_turtles_;

    rclcpp::Service<interfaces_pkg::srv::CatchTurtle>::SharedPtr catch_turtle_service_;
    rclcpp::Publisher<interfaces_pkg::msg::TurtleArray>::SharedPtr alive_turtles_publisher_;
    rclcpp::TimerBase::SharedPtr spawn_turtle_timer_;
    rclcpp::TimerBase::SharedPtr publish_alive_turtles_timer_;

    void callSpawnTurtleService(double x, double y, double theta)
    {
        auto client = this->create_client<turtlesim::srv::Spawn>("spawn");

        while (!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for Spawn Service...");
        }

        auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
        request->x = x;
        request->y = y;
        request->theta = theta;

        auto future = client->async_send_request(request);

        try
        {
            auto response = future.get();
            if (!response->name.empty())
            {
                interfaces_pkg::msg::Turtle new_turtle;
                new_turtle.name = response->name;
                new_turtle.x = x;
                new_turtle.y = y;
                new_turtle.theta = theta;
                alive_turtles_.push_back(new_turtle);

                publishAliveTurtles();
                RCLCPP_INFO(this->get_logger(), "Turtle %s spawned.", response->name.c_str());
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to call Spawn Service: %s", e.what());
        }
    }

    void callKillTurtleService(const std::string &turtle_name)
    {
        auto client = this->create_client<turtlesim::srv::Kill>("kill");

        while (!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for Kill Service...");
        }

        auto request = std::make_shared<turtlesim::srv::Kill::Request>();
        request->name = turtle_name;

        auto future = client->async_send_request(request);

        try
        {
            future.get();

            auto it = std::remove_if(
                alive_turtles_.begin(),
                alive_turtles_.end(),
                [&turtle_name](const interfaces_pkg::msg::Turtle &turtle)
                { return turtle.name == turtle_name; });

            if (it != alive_turtles_.end())
            {
                alive_turtles_.erase(it, alive_turtles_.end());
                publishAliveTurtles();
                RCLCPP_INFO(this->get_logger(), "Turtle %s killed.", turtle_name.c_str());
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to call Kill Service: %s", e.what());
        }
    }

    void callbackCatchTurtle(const interfaces_pkg::srv::CatchTurtle::Request::SharedPtr request,
                             const interfaces_pkg::srv::CatchTurtle::Response::SharedPtr response)
    {
        kill_turtle_threads_.push_back(
            std::make_shared<std::thread>(
                &TurtleSpawnNode::callKillTurtleService, this, request->name));
        response->success = true;
    }

    void spawnNewTurtle()
    {
        turtle_counter_++;
        double x = randomDouble() * 10.0;
        double y = randomDouble() * 10.0;
        double theta = randomDouble() * 2 * M_PI;

        spawn_turtle_threads_.push_back(
            std::make_shared<std::thread>(
                &TurtleSpawnNode::callSpawnTurtleService, this, x, y, theta));
    }

    double randomDouble()
    {
        return static_cast<double>(std::rand()) / (RAND_MAX + 1.0);
    }

    void publishAliveTurtles()
    {
        auto msg = interfaces_pkg::msg::TurtleArray();
        msg.turtles = alive_turtles_;
        alive_turtles_publisher_->publish(msg);
    }

public:
    TurtleSpawnNode() : Node("turtle_spawn_node"), turtle_counter_(0)
    {
        this->declare_parameter("turtle_name_prefix", "turtle");
        this->declare_parameter("spawn_frequency", 0.33);
        
        turtle_name_prefix_ = this->get_parameter("turtle_name_prefix").as_string();
        
        // spawn_frequency_ = this->get_parameter("spawn_frequency").as_double();

        turtle_name_prefix_ = "turtle";

        alive_turtles_publisher_ = this->create_publisher<interfaces_pkg::msg::TurtleArray>("alive_turtles", 10);

        publish_alive_turtles_timer_ = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&TurtleSpawnNode::publishAliveTurtles, this));

        spawn_turtle_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(2000), std::bind(&TurtleSpawnNode::spawnNewTurtle, this));

        catch_turtle_service_ = this->create_service<interfaces_pkg::srv::CatchTurtle>(
            "catch_turtle", std::bind(&TurtleSpawnNode::callbackCatchTurtle, this, std::placeholders::_1, std::placeholders::_2));
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleSpawnNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
