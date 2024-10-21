#include "rclcpp/rclcpp.hpp"
#include "dd/srv/patrol.hpp"
#include <chrono>
#include <ctime>
// #include "/home/zht/dev_ws/install/demo_cpp_pkg/include/demo_cpp_pkg/srv/patrol.hpp"
using Patrol = dd::srv::Patrol;
using namespace std::chrono_literals;//可以使用10s，2000ms这样的方法来表示时间

class PatrolClient: public rclcpp::Node
{
private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Client<Patrol>::SharedPtr patrol_client_;
public:
    PatrolClient(const std::string& node_name):Node(node_name){
        srand(time(NULL));
        patrol_client_ = this->create_client<Patrol>("patrol");
        timer_ = this->create_wall_timer(5s, [&]()->void{
            //检测是否连接上服务
            while (!this->patrol_client_->wait_for_service(1s)){
                if (!rclcpp::ok()){
                    RCLCPP_ERROR(this->get_logger(),"client interrupted");
                    return;
                }
                RCLCPP_INFO(this->get_logger()," waiting for service...");
                
            };
            //创建请求对象
            auto request = std::make_shared<Patrol::Request>();
            request->target_x = rand()%12;
            request->target_y = rand()%12;
            RCLCPP_INFO(this->get_logger(),"OK了,目标坐标(%f,%f)",request->target_x,request->target_y);
            //发送请求
            this->patrol_client_->async_send_request(request,[&](rclcpp::Client<Patrol>::SharedFuture result_future)->void{
                auto response = result_future.get();
                if(response->result==Patrol::Response::SUCCESS){
                    RCLCPP_INFO(this->get_logger(),"到了，到了");
                }
                else{
                    RCLCPP_INFO(this->get_logger(),"失败了，失败了");
                }
            }
                );//异步发送请求

    });
};
};
int main(int argc,char** argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<PatrolClient>("patrol_client");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
};
