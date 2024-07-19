#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;
using namespace std;

ofstream foutC1, foutC2;

class EvoTopic2Tum : public rclcpp::Node
{
public:
    EvoTopic2Tum()
    : Node("evo_topic2tum")
    {
        foutC1.open("./robot_pose.txt");
        foutC2.open("./ground_truth_pose.txt");
        //robot_pose from tf
        subscription_robot_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/robot_pose", 10, std::bind(&EvoTopic2Tum::callback1, this, _1));
        //ground truth from gazebo 
        subscription_ground_truth_pose_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/gazebo_p3d/ground_truth/pose", 10, std::bind(&EvoTopic2Tum::callback2, this, _1));
    }

private:
    void callback1(const geometry_msgs::msg::PoseStamped & pos) const
    {
        //receive robot_pose from topic
        foutC1 << pos.header.stamp.sec<<"."<<pos.header.stamp.nanosec << " ";
        float x = pos.pose.position.x;
        float y = pos.pose.position.y;
        float z = pos.pose.position.z;
        float qx = pos.pose.orientation.x;
        float qy = pos.pose.orientation.y;
        float qz = pos.pose.orientation.z;
        float qw = pos.pose.orientation.w;
        RCLCPP_INFO(this->get_logger(), "robot pose: %f %f %f %f %f %f %f",x,y,z,qx,qy,qz,qw);
        foutC1 << x <<" " << y << " " << z << " " << qx << " " << qy << " " << qz << " " << qw << std::endl;

    }
    void callback2(const nav_msgs::msg::Odometry & truth) const
    {
       //receive gazebo_p3d/ground_truth/pose from topic 
        foutC2 << truth.header.stamp.sec<<"."<<truth.header.stamp.nanosec << " ";
        float x2 = truth.pose.pose.position.x;
        float y2 = truth.pose.pose.position.y;
        float z2 = truth.pose.pose.position.z;
        float qx2 = truth.pose.pose.orientation.x;
        float qy2 = truth.pose.pose.orientation.y;
        float qz2 = truth.pose.pose.orientation.z;
        float qw2 = truth.pose.pose.orientation.w;
        RCLCPP_INFO(this->get_logger(), "ground truth: %f %f %f %f %f %f %f",x2,y2,z2,qx2,qy2,qz2,qw2);
        foutC2 << x2 <<" " << y2 << " " << z2 << " " << qx2 << " " << qy2 << " " << qz2 << " " << qw2 << std::endl;
    }
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_robot_pose_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_ground_truth_pose_;


};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EvoTopic2Tum>());
  rclcpp::shutdown();
  foutC1.close();
  foutC2.close();
  return 0;
}