#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/srv/spawn.hpp"
#include "turtlesim/msg/pose.hpp"
#include "turtlesim/srv/kill.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <cstdlib>
#include <string>
#include <ctime>
#include <cmath>
#include <limits>
#include <vector>
#include <algorithm>
using namespace std::chrono_literals;
struct Turtle{
    std::string name;
    float x_pos;
    float y_pos;
};
class Controller_Node: public rclcpp::Node{
    public:
        Controller_Node():Node("controller_node"){
            srand(time(0));
            spawn_client_ = this->create_client<turtlesim::srv::Spawn>("/spawn");
            generator_timer_ = this->create_wall_timer(4000ms,std::bind(&Controller_Node::turtle_generator, this));
            publisher_timer_ = this->create_wall_timer(5ms,std::bind(&Controller_Node::position, this));
            position_subscriber_ = this->create_subscription<turtlesim::msg::Pose>("/turtle1/pose",10,std::bind(&Controller_Node::position_callback,this,std::placeholders::_1));
            velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
            kill_client_ = this->create_client<turtlesim::srv::Kill>("/kill");
        }
    private:
        void turtle_generator(){
            if (spawn_pending) {
                return;
            }
            if (!spawn_client_->service_is_ready()){
                RCLCPP_WARN(this->get_logger(),"Spawn service is not available yet.");
                return;
            }
            float rand_x = 1.0f + static_cast<float>(rand()) / RAND_MAX * 9.0f;
            float rand_y = 1.0f + static_cast<float>(rand()) / RAND_MAX * 9.0f;
            auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
            request->x = rand_x;
            request->y = rand_y; 
            request->theta = 0.0;
            request->name = "";
            //RCLCPP_INFO(this->get_logger(),"Requesting turtle at x= %f, y= %f",request->x,request->y);
            target_x = rand_x;
            target_y = rand_y;
            spawn_pending = true;
            auto result = spawn_client_->async_send_request(request,std::bind(&Controller_Node::spawn_response_callback,this,std::placeholders::_1));
        }
        void position(){
            velocity_publisher_->publish(output);
        }
        void position_callback(const turtlesim::msg::Pose::SharedPtr position){
            int index = find_closest_turtle(position->x , position->y);
            if(index <0 ){
                output.linear.x = 0.0;
                output.angular.z = 0.0;
                return;
            }
            Turtle target = turtles_[index];
            distance_error = std::sqrt((target.x_pos-position->x)*(target.x_pos-position->x)+(target.y_pos-position->y)*(target.y_pos - position->y));
            if(distance_error < catch_distance){
                kill(target.name);
                kill_pending = false;
                turtles_.erase(turtles_.begin()+index);
                return;
            }
            float desired_theta = std::atan2((target.y_pos-position->y),(target.x_pos-position->x));
            angular_error = std::atan2(std::sin(desired_theta-position->theta),std::cos(desired_theta-position->theta));
            //float d_term_l = kp1 * distance_error;
            //float d_term_a = kp2 * angular_error; 
            // output.linear.x = std::clamp(d_term_l,0.0f,100.0f);
            // output.angular.z = std::clamp(d_term_a,0.0f,100.0f);
            float d = kp1 * distance_error;
            float a = kp2 * angular_error;
            output.linear.x  = std::clamp(d,0.0f,2.5f);
            output.angular.z = std::clamp(a,-3.0f,3.0f);
            //velocity_publisher_->publish(output);
        }
        void spawn_response_callback(rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture future){
            auto response = future.get();
            turtles_.push_back({response->name,target_x,target_y});
            spawn_pending = false;
        }
        int find_closest_turtle(float master_x, float master_y){
            if(turtles_.empty()) return -1;
            float min_distance = std::numeric_limits<float>::max();
            int closest_turtle = 0;
            for (int i = 0; i < turtles_.size(); i++){
                float distance = (turtles_[i].x_pos-master_x)*(turtles_[i].x_pos-master_x)+(turtles_[i].y_pos-master_y)*(turtles_[i].y_pos-master_y);
                if(distance < min_distance){
                    min_distance = distance;
                    closest_turtle = i;
                }
            }
            return closest_turtle;
        }
        void kill(const std::string& name){
            if(kill_pending) return;
            if(!kill_client_->service_is_ready()){
                RCLCPP_WARN(this->get_logger(),"Kill service is not available yet.");
                return;
            }
            auto request = std::make_shared<turtlesim::srv::Kill::Request>();
            request->name = name;
            kill_pending = true;
            auto result = kill_client_->async_send_request(request);
        }
        std::vector<Turtle> turtles_;
        const float catch_distance = 0.001f;
        bool spawn_pending{false};
        bool kill_pending{false};
        float error_x {0.0f};
        float error_y {0.0f};
        float target_x {0.0f} , target_y {0.0f};
        float distance_error{0.0f};
        float angular_error{0.0f};
        std::string turtle_name;
        float kp1 {2.0f}, ki1{0.0f} , kd1{0.0f};
        float kp2 {2.5f}, ki2{0.0f} , kd2{0.0f};
        rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawn_client_;
        rclcpp::TimerBase::SharedPtr generator_timer_;
        rclcpp::TimerBase::SharedPtr publisher_timer_;
        rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr position_subscriber_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
        rclcpp::Client<turtlesim::srv::Kill>::SharedPtr kill_client_;
        geometry_msgs::msg::Twist output;

};
int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Controller_Node>());
  rclcpp::shutdown();
  return 0;
}