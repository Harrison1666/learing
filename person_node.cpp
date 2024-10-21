#include "rclcpp/rclcpp.hpp"

//PersonsNode继承了rclcpp::Node类
class PersonNode : public rclcpp::Node
{
private:
    std::string name_;
    int age_;

public:
    PersonNode(const std::string &node_name, const std::string &name, const int &age) 
    : Node(node_name)//调用父类的构造函数
    {
        this->name_ = name;
        this->age_ = age;
    }
    
    void eat(const std::string& food)
    {
        RCLCPP_INFO(this->get_logger(), "I am %s, ,I am %d years old, I eat %s", this->name_.c_str(), this->age_, food.c_str());//打印日志信息
    }

};

int main(int argc,char**argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<PersonNode>("person_node","Harrison",19);
    RCLCPP_INFO(node->get_logger(),"Harrison is handsome");
    node->eat("鱼香肉丝");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}