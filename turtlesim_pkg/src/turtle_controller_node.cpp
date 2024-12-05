#include "rclcpp/rclcpp.hpp"

#include "turtlesim/msg/pose.hpp" 
#include "geometry_msgs/msg/twist.hpp"

#include "interfaces_pkg/msg/turtle.hpp"
#include "interfaces_pkg/msg/turtle_array.hpp"

#include "interfaces_pkg/srv/catch_turtle.hpp"

//create a subscriber that take element by topic turtlesim_pose
    //first of all a callback function to take the pose

    //for cmd_vel use interface geometry_msgs
class TurtleControllerNode : public rclcpp::Node
{

private: 

    // float target_x_,target_y_; 
    // float x_, y_, theta_;
    // float linear_velocity_, angular_velocity_;

    turtlesim::msg::Pose pose_;
    interfaces_pkg::msg::Turtle turtle_to_catch_;
    bool turtlesim_up_;
    bool catch_closest_turtle_first_;

    // rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr turtle_vel_sub_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr turtle_pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr turtle_vel_pub_;
    rclcpp::TimerBase::SharedPtr control_loop_timer_;
    rclcpp::Subscription<interfaces_pkg::msg::TurtleArray>::SharedPtr turtles_to_catch_subscriber_;


    std::vector<std::shared_ptr<std::thread>> catch_turtle_threads;

    void callback_turtle_pose(const turtlesim::msg::Pose::SharedPtr msg ){
        // x_ = msg->x;
        // y_ = msg->y;
        // theta_ = msg->theta;
        // pose_ = *msg.get(); //puntiamo qui!!!!!!!!!
        // turtlesim_up_ = true;
        // RCLCPP_INFO(this-> get_logger(),"Posizione - x: %.2f, y: %.2f, theta: %.2f", x_, y_, theta_);
        pose_ = *msg.get();
        turtlesim_up_ = true;
    }

    // void callback_turtle_vel(const geometry_msgs::msg::Twist::SharedPtr msg){
    //     linear_velocity_ = msg->linear.x;
    //     angular_velocity_ = msg->linear.z;
    //     RCLCPP_INFO(this->get_logger(), "Velocità - lineare: %.2f, angolare: %.2f", linear_velocity_, angular_velocity_);
    // }

  

    void callbackTurtlesToCatch(const interfaces_pkg::msg::TurtleArray::SharedPtr msg)
    {
        if (!msg->turtles.empty())
        {
            if (catch_closest_turtle_first_)
            {
                interfaces_pkg::msg::Turtle closest_turtle = msg->turtles.at(0);
                double closest_turtle_distance = getDistanceFromCurrentPose(closest_turtle);

                for (int i = 1; i < (int)msg->turtles.size(); i++)
                {
                    double distance = getDistanceFromCurrentPose(msg->turtles.at(i));
                    if (distance < closest_turtle_distance)
                    {
                        closest_turtle = msg->turtles.at(i);
                        closest_turtle_distance = distance;
                    }
                }

                turtle_to_catch_ = closest_turtle;
            }
            else
            {
                turtle_to_catch_ = msg->turtles.at(0);
            }
        }
    }

    void publishVel(double x, double theta)
    {
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = x;
        msg.angular.z = theta;
        turtle_vel_pub_->publish(msg);
    }


    void control_loop()
    {
        if (!turtlesim_up_ || turtle_to_catch_.name == "")
        {
            return;
        }

        double dist_x = turtle_to_catch_.x - pose_.x;
        double dist_y = turtle_to_catch_.y - pose_.y;
        double distance = std::sqrt(dist_x * dist_x + dist_y * dist_y);

        auto msg = geometry_msgs::msg::Twist();

        if (distance > 0.5)
        {
            // position
            msg.linear.x = 2 * distance;

            // orientation
            double steering_angle = std::atan2(dist_y, dist_x);
            double angle_diff = steering_angle - pose_.theta;
            if (angle_diff > M_PI)
            {
                angle_diff -= 2 * M_PI;
            }
            else if (angle_diff < -M_PI)
            {
                angle_diff += 2 * M_PI;
            }
            msg.angular.z = 6 * angle_diff;
        }
        else
        {
            // target reached!
            msg.linear.x = 0.0;
            msg.angular.z = 0.0;
            
            catch_turtle_threads.push_back(
                std::make_shared<std::thread>(
                    std::bind(&TurtleControllerNode::callCatchTurtleService, this, turtle_to_catch_.name)));
            turtle_to_catch_.name = "";
        }

        turtle_vel_pub_->publish(msg);
    }

    
    double getDistanceFromCurrentPose(interfaces_pkg::msg::Turtle turtle)
    {
        double dist_x = turtle.x - pose_.x;
        double dist_y = turtle.y - pose_.y;
        return std::sqrt(dist_x * dist_x + dist_y * dist_y);
    }

    
    void callCatchTurtleService(std::string turtle_name)
    {
        auto client = this->create_client<interfaces_pkg::srv::CatchTurtle>("catch_turtle");
        while (!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for Service Server to be up...");
        }

        auto request = std::make_shared<interfaces_pkg::srv::CatchTurtle::Request>();
        request->name = turtle_name;

        auto future = client->async_send_request(request);

        try
        {
            auto response = future.get();
            if (!response->success)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to catch turtle");
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed.");
        }
    }

public:

    TurtleControllerNode() : Node("Turtle_controller_node") 
    {
        this->declare_parameter("catch_closest_turtle_first", true);
        catch_closest_turtle_first_ = this->get_parameter("catch_closest_turtle_first").as_bool();
        
        // target_x_ = 8.0;
        // target_y_ = 4.0;

        // turtle_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        //     "turtle1/cmd_vel", 10, std::bind(&TurtleControllerNode::callback_turtle_vel, this, std::placeholders::_1));

        turtle_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
        
      
        turtle_pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
            "turtle1/pose", 10, std::bind(&TurtleControllerNode::callback_turtle_pose, this, std::placeholders::_1));

        control_loop_timer_ = this-> create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&TurtleControllerNode::control_loop,this));
        
        turtles_to_catch_subscriber_ = this->create_subscription<interfaces_pkg::msg::TurtleArray>(
             "alive_turtles", 10, std::bind(&TurtleControllerNode::callbackTurtlesToCatch, this, std::placeholders::_1));
    }   

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleControllerNode>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
