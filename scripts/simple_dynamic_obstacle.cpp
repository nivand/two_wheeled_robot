#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <gazebo_msgs/srv/set_entity_state.hpp>
#include <chrono>
#include <cstdlib>
#include <memory>
#include<chrono> 
#include<random> 
#add code for getting the dynamic obj positions
using namespace std; 
using namespace std::chrono_literals;

class DynamicObstacle : public rclcpp::Node
{
    public:
    DynamicObstacle()
        : Node("simple_dynamic_obstacle_node")
    {
        client = this->create_client<gazebo_msgs::srv::SetEntityState>("/set_entity_state");
        obs_state.name = "box1";
        obs_state.pose.position.z = 0.5;
        obs_state.pose.orientation.x = 0;
        obs_state.pose.orientation.y = 0;
        obs_state.pose.orientation.z = 0;
        obs_state.pose.orientation.w = 1;

        this->declare_parameter<double>("v_x");
        this->declare_parameter<double>("v_y");
        this->declare_parameter<std::string>("model_name");
        this->declare_parameter<double>("min_x");
        this->declare_parameter<double>("min_y");
        this->declare_parameter<double>("max_x");
        this->declare_parameter<double>("max_y");

        this->get_parameter("v_x", vx_);
        this->get_parameter("v_y", vy_);
        this->get_parameter("model_name", obs_state.name);
        this->get_parameter("min_x", minx_);
        this->get_parameter("min_y", miny_);
        this->get_parameter("max_x", maxx_);
        this->get_parameter("max_y", maxy_);

        obs_state.pose.position.x = minx_;
        obs_state.pose.position.y = miny_;

        timer_ = this->create_wall_timer(
      50ms, std::bind(&DynamicObstacle::timer_callback, this));

    }

    private:
    void timer_callback()
    {  
        double a,b;
        a=0.04;
        b=0.02;
        vx_ += a*0.05 ;
        vy_ += b*0.05;
        obs_state.pose.position.x += vx_*0.05 + 0.5* a *0.0025 ;
        obs_state.pose.position.y += vy_*0.05 + 0.5* b *0.0025;
        
        if (obs_state.pose.position.x > minx_ + 2.0)
        {
            a = -0.001;
        }
        if (obs_state.pose.position.y > miny_ + 0.7)
        {
           
            b = -0.001;
        }

        if (!initial_state)
        {
            if ((obs_state.pose.position.x < minx_) || (obs_state.pose.position.x > maxx_))
                vx_ = -vx_;
                a = -0.01;
            if ((obs_state.pose.position.y < miny_) || (obs_state.pose.position.y > maxy_))
                vy_ = -vy_;
                b = -0.01;
        }
        
        initial_state = false;
        gazebo_msgs::srv::SetEntityState::Request::SharedPtr state_req = std::make_shared<gazebo_msgs::srv::SetEntityState::Request>();
        state_req->state = obs_state;
        auto result = client->async_send_request(state_req);
        
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Client<gazebo_msgs::srv::SetEntityState>::SharedPtr client;
    gazebo_msgs::msg::EntityState obs_state;
    double vx_, vy_, minx_, miny_, maxx_, maxy_ , past ,diff ;
    bool initial_state = true;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DynamicObstacle>());
  rclcpp::shutdown();
  return 0;
}
