#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/srv/spawn.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <cstdlib>
using namespace std::chrono_literals;
class Controller_Node: public rclcpp::Node{
    public:
        Controller_Node():Node("controller_node"){
            srand(time(0));
            spawn_client_ = this->create_client<turtlesim::srv::Spawn>("/spawn");
            generator_timer_ = this->create_wall_timer(3000ms,std::bind(&Controller_Node::turtle_generator, this));
        }
    private:
        void turtle_generator(){
            if (!spawn_client_->service_is_ready()){
                RCLCPP_WARN(this->get_logger(),"Spawn service is not available yet.");
                return;
            }
            float rand_x = rand() % 12;
            float rand_y = rand() % 12;
            auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
            request->x = rand_x;
            request->y = rand_y; 
            request->theta = 0.0;
            request->name = "";
            RCLCPP_INFO(this->get_logger(),"Requesting turtle at x= %f, y= %f",request->x,request->y);
            auto result = spawn_client_->async_send_request(request);
        }

        rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawn_client_;
        rclcpp::TimerBase::SharedPtr generator_timer_;
};
int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Controller_Node>());
  rclcpp::shutdown();
  return 0;
}