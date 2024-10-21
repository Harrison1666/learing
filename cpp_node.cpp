#include"rclcpp/rclcpp.hpp"


int main(int argc, char **argv)
{
        rclcpp::init(argc, argv);//初始化ROS节点
        auto node = std::make_shared<rclcpp::Node>("cpp_node");//创建节点
        RCLCPP_INFO(node->get_logger(), "阿Hellos, world!");
        rclcpp::spin(node);
        rclcpp::shutdown();
        return 0;
}