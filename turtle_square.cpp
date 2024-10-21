#include <chrono>  
#include "rclcpp/rclcpp.hpp"  
#include "geometry_msgs/msg/twist.hpp"  
#include <thread>//多线程库  
#include <cmath>
using namespace std::chrono_literals;

class TurtleSquare : public rclcpp::Node 
{  
    private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;//发布者的智能指针
    rclcpp::TimerBase::SharedPtr timer_;    
    int length;  
    int turn_duration_;  
    double pi=M_PI;

public:  
    explicit TurtleSquare(const std::string& node_name):Node(node_name) 
    {  
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);  
        timer_ = this->create_wall_timer(1000ms, std::bind(&TurtleSquare::timer_callback, this));  

        // 设置边长和旋转速度 
        length= 8;  // 海龟每次移动的距离（米）  
        turn_duration_ = 2; // 旋转90度所需的时间
    } 

    void timer_callback() 
    {   
            auto msg=geometry_msgs::msg::Twist();  

            // 向前移动  
            msg.linear.x = 2.0;  // 线速度  
            msg.angular.z = 0.0;  // 角速度  
            publisher_->publish(msg);  

            // 等待足够的时间让海龟移动  
            std::this_thread::sleep_for(std::chrono::seconds(turn_duration_)); 

            // 停止移动  
            msg.linear.x = 0.0;  
            publisher_->publish(msg);  

            // 旋转90度  
            msg.angular.z = M_PI/2;  // 旋转速度，可能需要调整以达到90度  
            publisher_->publish(msg);  

            // 等待旋转完成  
            std::this_thread::sleep_for(std::chrono::seconds(turn_duration_));  

            // 重置角速度  
            msg.angular.z = 0.0;  
            publisher_->publish(msg);  
        
    }  
};  
int main(int argc, char **argv) 
{  
    rclcpp::init(argc,argv);
    auto node = std::make_shared<TurtleSquare>("turtle_square");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}