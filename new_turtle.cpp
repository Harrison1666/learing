#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <chrono>
#include "turtlesim/msg/pose.hpp"
#include "dd/srv/patrol.hpp"
// #include "/home/zht/dev_ws/install/demo_cpp_pkg/include/demo_cpp_pkg/srv/patrol.hpp"
using Patrol = dd::srv::Patrol;
using namespace std::chrono_literals;

class TurtleControlNode: public rclcpp::Node
{
private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;//发布者的智能指针
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscriber_;//订阅者的智能指针
    rclcpp::Service<Patrol>::SharedPtr patrol_service_;
    double target_x{1.0};
    double target_y{3.0};
    double k{1.0};//控制速度的比例系数
    double max_speed{2.0};//最大速度

public:
    //引式转换：如果这个类的构造函数只有一个参数，那么就可以通过 类 变量名=这一个参数的参数值 来构造出这个类
    //为防止该情况，使用explicit 
    explicit TurtleControlNode(const std::string& node_name):Node(node_name)
    {
        patrol_service_=this->create_service<Patrol>("patrol",[&](const std::shared_ptr<Patrol::Request> request,
        std::shared_ptr<Patrol::Response> response)->void{
        if((0<request->target_x && request->target_x<12.0f)&&
        (0<request->target_y && request->target_y<12.0f))
        {
            this->target_x =request->target_x;
            this->target_y =request->target_y;
            response->result =Patrol::Response::SUCCESS;
        }
        else
        {response->result =Patrol::Response::FALSE;}
        });
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel",10);
        subscriber_ = this->create_subscription<turtlesim::msg::Pose>("/turtle1/pose",10,
        std::bind(&TurtleControlNode::on_pose_received,this,std::placeholders::_1));

    }

    void on_pose_received(const turtlesim::msg::Pose::SharedPtr pose)//参数：收到数据的共享指针
    {
        //1.获取当前位置
        auto current_x = pose->x;
        auto current_y = pose->y;
        RCLCPP_INFO(this->get_logger(),"Current position: x=%f, y=%f",current_x,current_y);

        //2.计算目标位置
        auto distance = sqrt((target_x - current_x)*(target_x - current_x) + (target_y - current_y)*(target_y - current_y));
        auto angle = std::atan2(target_y - current_y, target_x - current_x) - pose->theta;//计算出目标点与当前点的角度

        //3.控制策略
        auto msg = geometry_msgs::msg::Twist() ;
        if(distance > 0.1)
        {
            if(fabs(angle) > 0.2){
                msg.angular.z=fabs(angle);
            }
            else {
                msg.linear.x=k*distance;
            }
        
        }
        //4.限制最大速度
        if(msg.linear.x > max_speed){
            msg.linear.x = max_speed;
        }

        publisher_->publish(msg);
    };

};

int main(int argc,char** argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<TurtleControlNode>("turtle_circle");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
