#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <chrono>

using namespace std::chrono_literals;

class TurtleCircleNode: public rclcpp::Node
{
private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;//发布者的智能指针
    rclcpp::TimerBase::SharedPtr timer_;

public:
    //引式转换：如果这个类的构造函数只有一个参数，那么就可以通过 类 变量名=这一个参数的参数值 来构造出这个类
    //为防止该情况，使用explicit 
    explicit TurtleCircleNode(const std::string& node_name):Node(node_name)
    {
        publisher_=this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel",10);
        timer_=this->create_wall_timer(1000ms,std::bind(&TurtleCircleNode::timer_callback,this));

    }

    void timer_callback(){
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x=1.0;
        msg.angular.z=0.5;
        publisher_->publish(msg);
    }

};

int main(int argc,char** argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<TurtleCircleNode>("turtle_circle");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}