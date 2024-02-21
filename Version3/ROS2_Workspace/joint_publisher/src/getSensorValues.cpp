#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <math.h>

using std::placeholders::_1;

class getSensorValues: public rclcpp::Node{
    public:
        getSensorValues():Node("getSensorValues"){
            this->sensor = this->create_subscription<sensor_msgs::msg::Imu>("imu_plugin1/out",50,std::bind(&getSensorValues::sensorCallBack,this, _1));
            this->sensorPub = this->create_publisher<std_msgs::msg::Float64>("/sensorPub", 50);

        }

        void sensorCallBack(const sensor_msgs::msg::Imu::SharedPtr value){
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"%.5f", value->linear_acceleration.y);

        }
    
    private:
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sensor;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr sensorPub;
        std_msgs::msg::Float64 msg;

};

int main(int argc, char **argv){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<getSensorValues>());
    rclcpp::shutdown();
    return 0;
}


