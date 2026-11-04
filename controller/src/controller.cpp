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
            generator_timer_ = this->create_wall_timer(3000ms,std::bind(&Controller_Node::turtle_generator, this));
            publisher_timer_ = this->create_wall_timer(10ms,std::bind(&Controller_Node::v_publisher, this));
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
        void v_publisher(){
            velocity_publisher_->publish(output);
        }
        void position_callback(const turtlesim::msg::Pose::SharedPtr position){
            start_ = std::chrono::steady_clock::now();
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
                RCLCPP_INFO(this->get_logger(),"distance error: %f , linear velocity: %f", distance_error,output.linear.x);
                return;
            }
            float desired_theta = std::atan2((target.y_pos-position->y),(target.x_pos-position->x));
            angular_error = std::atan2(std::sin(desired_theta-position->theta),std::cos(desired_theta-position->theta));
            //angular_error = desired_theta - position->theta;
            float d = kp_v * distance_error;
            float a = kp_a * angular_error;
            integral_v = distance_error * 0.01f;
            integral_a = angular_error * 0.01f;
            d += integral_v* ki_v;
            a += integral_a * ki_a;
            derivative_v= (distance_error - prev_error_v)/ 0.01f; 
            derivative_a = (angular_error - prev_error_a)/0.01f;
            d += derivative_v * kd_v;
            a+= derivative_a * kd_a;
            output.linear.x  = std::clamp(d,0.0f,4.0f);
            output.angular.z = std::clamp(a,-3.1415f,3.1415f);
            prev_error_v = distance_error;
            prev_error_a = angular_error;
            //RCLCPP_INFO(this->get_logger(),"linear: %f , angular: %f", output.linear.x,output.angular.z);
            auto now_ = std::chrono::steady_clock::now();
            float dt = std::chrono::duration<float>(now_ - start_).count();
            //RCLCPP_INFO(this->get_logger(),"controller time: %f",dt);
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
            auto result = kill_client_->async_send_request(request);
            kill_pending = true;
        }
        std::vector<Turtle> turtles_;
        bool spawn_pending{false};
        bool kill_pending{false};
        float error_x {0.0f};
        float error_y {0.0f};
        float target_x {0.0f} , target_y {0.0f};
        float integral_v{0.0f}, integral_a{0.0f};
        float derivative_v{0.0f} , derivative_a{0.0f};
        float prev_error_v {0.0f} , prev_error_a{0.0f};
        float distance_error{0.0f};
        float angular_error{0.0f};
        std::string turtle_name;
        float kp_v {2.50f}, ki_v{0.5f} , kd_v{0.25f};
        float kp_a {4.0f}, ki_a{0.0f} , kd_a{0.0f};
        const float catch_distance = 0.025f;
        std::chrono::steady_clock::time_point start_ ;
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