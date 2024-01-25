#include <rclcpp/rclcpp.hpp>
#include "gen3_controller/data_container.h"
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include "geometry_msgs/msg/wrench_stamped.hpp"  

#ifndef Gen3Interface_H
#define Gen3Interface_H
class Gen3Interface
{
    public:
        Gen3Interface(rclcpp::Node::SharedPtr node, DataContainer &dc);
        ~Gen3Interface();
        void stateUpdate();
        void simStatusCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
        void ft_StatusCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);
        void pubCommand();
        void gen3Command();

    private:
        DataContainer &dc_;
        std::mutex m_dc_;
        std::mutex m_ci_;
        rclcpp::Node::SharedPtr node_; // 여기에 추가

        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr gen3_status_sub_;
        rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr gen3_ft_sub_;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr gen3_pos_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gen3_joint_cmd_pub_;



        bool is_first_callback = true;
        bool is_first_ft = true;

        

        sensor_msgs::msg::JointState gen3_pos_msg_;
        std_msgs::msg::Float64MultiArray gen3_joint_cmd_msg_; 

};
#endif