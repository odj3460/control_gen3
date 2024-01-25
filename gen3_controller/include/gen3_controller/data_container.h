#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

#ifndef DataContainer_H
#define DataContainer_H
class DataContainer
{
    public:
        rclcpp::Node::SharedPtr node;

        bool is_first_callback = true;

        rclcpp::Time sim_timestamp_;
        double sim_time_;
        std::string sim_mode_ = "torque";

        int num_dof_;

        Eigen::VectorXd q_;
        Eigen::VectorXd q_dot_;
        Eigen::VectorXd effort_;
        Eigen::VectorXd ft_;
        Eigen::VectorXd ftt_;
        Eigen::VectorXd g_;

        Eigen::Isometry3d x_;
        Eigen::MatrixXd j_;

        Eigen::VectorXd control_input_;
};
#endif