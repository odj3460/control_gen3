#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <chrono>
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "gen3_controller/gen3_controller.h"
#include <ament_index_cpp/get_package_share_directory.hpp>



Gen3Controller::Gen3Controller(rclcpp::Node::SharedPtr node, DataContainer &dc) : dc_(dc)
{
    dc_.sim_mode_ = "position";
    
    std::cout << "Robot Model Memory Address: " << &right_arm_.robot_model_ << std::endl;

    // RBDL
    std::string urdf_name = ament_index_cpp::get_package_share_directory("kortex_description") + "/arms/gen3/7dof/urdf/gen3_7d.urdf";
    std::cout << "Robot Model name: " << urdf_name << std::endl;
    RigidBodyDynamics::Addons::URDFReadFromFile(urdf_name.c_str(), &right_arm_.robot_model_, false, false);


    // txt_right_.open("/home/camasodj/dj_ws/src/data_right.txt", std::ios::out); 
    // if (!txt_right_.is_open())
    // {
    //     std::cout << "Unable to open right txt file" << std::endl;
    //     return;
    // }


}

Gen3Controller::~Gen3Controller()
{

}


void Gen3Controller::compute()
{
    rclcpp::Rate r(1000);
    while (rclcpp::ok())
    {
        // if (!dc_.is_first_callback)
        // {
        //     if (!is_init_)
        //     {

        //         right_arm_.q_.resize(DOF);
        //         right_arm_.q_.setZero();
        //         right_arm_.q_dot_.resize(DOF);
        //         right_arm_.q_dot_.setZero();
        //         right_arm_.q_dot_zero_.resize(DOF);
        //         right_arm_.q_dot_zero_.setZero();
                
        //         right_arm_.effort_.resize(DOF);
        //         right_arm_.effort_.setZero();
        //         right_arm_.control_input_.resize(DOF);
        //         right_arm_.control_input_.setZero();
                
        //         right_arm_.q_desired.resize(DOF);
        //         right_arm_.q_desired.setZero();
        //         right_arm_.q_init_.resize(DOF);
        //         right_arm_.q_init_.setZero();

        //         right_arm_.j_temp_.resize(6, DOF);
        //         right_arm_.j_temp_.setZero();
        //         right_arm_.j_.resize(6, DOF);
        //         right_arm_.j_.setZero();

        //         right_arm_.A_.resize(DOF, DOF);
        //         right_arm_.A_.setZero();
        //         right_arm_.g_.resize(DOF);
        //         right_arm_.g_.setZero();

        //         right_arm_.ft_.resize(6);
        //         right_arm_.ft_.setZero();
        //         right_arm_.x_dot_.resize(6);
        //         right_arm_.x_dot_.setZero();


        //         is_init_ = true;
        //     }

        //     m_dc_.lock();
        //     right_arm_.ft_before_ = right_arm_.ft_;
        //     right_arm_.ft_before_ = right_arm_.ft_;
        //     right_arm_.x_dot_before_ = right_arm_.x_dot_;
        //     sim_time_ = dc_.sim_time_;
        //     right_arm_.q_ = dc_.q_;
        //     right_arm_.q_dot_ = dc_.q_dot_;
        //     right_arm_.effort_ = dc_.effort_;
        //     right_arm_.ft_= dc_.ft_; 
        //     right_arm_.control_input_ = dc_.q_;
        //     m_dc_.unlock();

            // updateKinematicsDynamics();
            // computeControlInput();

            // right_arm_.x_init_.translation() = right_arm_.x_.translation();
            // right_arm_.x_init_.linear() = right_arm_.x_.linear();

            // control_start_time_ = sim_time_;


        // }

        // txt_right_ 
        // << sim_time_ << " " << control_start_time_<< " " 
        // // << right_arm_.x_.translation().transpose() << " " 
        // << right_arm_.control_input_.transpose() << " " 
        // << std::endl;

        computeControlInput();


        r.sleep();
    }

}

void Gen3Controller::updateKinematicsDynamics()
{

    static const int BODY_ID = right_arm_.robot_model_.GetBodyId("end_effector_link");

    // ---------------   Kinematoics
    right_arm_.x_.translation().setZero();
    right_arm_.x_.linear().setZero();
    right_arm_.x_.translation() = RigidBodyDynamics::CalcBodyToBaseCoordinates(right_arm_.robot_model_, right_arm_.q_, BODY_ID, Eigen::Vector3d(0.0, 0.0, 0.0), true);
    right_arm_.x_.linear() = RigidBodyDynamics::CalcBodyWorldOrientation(right_arm_.robot_model_, right_arm_.q_, BODY_ID, true).transpose();

    right_arm_.j_temp_.setZero();
    RigidBodyDynamics::CalcPointJacobian6D(right_arm_.robot_model_, right_arm_.q_, BODY_ID, Eigen::Vector3d(0.0, 0.0, 0.0), right_arm_.j_temp_, true);
    right_arm_.j_.setZero();

    for (int i = 0; i<2; i++)
    {
        right_arm_.j_.block<3, 7>(i * 3, 0) = right_arm_.j_temp_.block<3, 7>(3 - i * 3, 0);
    }  

    right_arm_.x_dot_ = right_arm_.j_ * right_arm_.q_dot_;

    RigidBodyDynamics::NonlinearEffects(right_arm_.robot_model_, right_arm_.q_, right_arm_.q_dot_zero_, right_arm_.g_);

}


void Gen3Controller::computeControlInput()
{
    // Task Space PD Gain
    Eigen::MatrixXd Kp_ = Eigen::MatrixXd::Zero(6, 6);

    for (int i = 0; i<3; i++)
    {
        Kp_(i,i) = 5;        // Translation
        Kp_(i+3,i+3) = 0.5;     // Rotation
    }


    // Eigen::Isometry3d x_target_;
    // x_target_.translation() << 0.426, 0.00, 0.356;
    // x_target_.linear() = right_arm_.x_.linear(); 
    // double duration_time = 10.0;

    // Eigen::MatrixXd j_inverse_;
    // j_inverse_.resize(7,6);
    // j_inverse_ = right_arm_.j_.transpose()*((right_arm_.j_*right_arm_.j_.transpose()).inverse());

    // // Translation
    // Eigen::Isometry3d x_cubic_;
    // Eigen::Vector6d x_cubic_dot_;
    // for (int i = 0; i < 3; i++)
    // {
    //     x_cubic_.translation()(i) = cubic(sim_time_, control_start_time_, control_start_time_ + duration_time, right_arm_.x_init_.translation()(i), x_target_.translation()(i), 0.0, 0.0);
    //     x_cubic_dot_(i) = cubicDot(sim_time_, control_start_time_, control_start_time_ + duration_time, right_arm_.x_init_.translation()(i), x_target_.translation()(i), 0.0, 0.0);
    // }
    // // Rotation
    // Eigen::Vector3d rotation_cubic_dot;
    // x_cubic_.linear() = rotationCubic(sim_time_, control_start_time_, control_start_time_ + duration_time, right_arm_.x_init_.linear(), x_target_.linear());
    // rotation_cubic_dot = rotationCubicDot(sim_time_, control_start_time_, control_start_time_ + duration_time, right_arm_.x_init_.linear(), x_target_.linear());

    // for (int i = 0; i < 3; i++)
    // {
    //     x_cubic_dot_(i + 3) = rotation_cubic_dot(i);
    // }
    // // Trans + Rot
    // Eigen::Vector6d x_error_;
    // x_error_ << x_cubic_.translation() - right_arm_.x_.translation(), -getPhi(right_arm_.x_.linear(), x_cubic_.linear());

    // Eigen::Vector7d q_desired_;
    // q_desired_ = right_arm_.q_ + j_inverse_ * (x_cubic_dot_+ Kp_* x_error_)/500.0;

    // // Control input 입력 
    // for (int i = 0; i <7; i++)
    // {
    //     right_arm_.control_input_(i) = q_desired_(i);
    // }

    // std::cout << " Rotation Matrix (NO MODE):  " << right_arm_.x_.linear().transpose() << std::endl;
    // std::cout << " Joint State (NO MODE):  " << right_arm_.q_.transpose() << std::endl;

    m_ci_.lock();
    // dc_.control_input_ = right_arm_.control_input_;
    dc_.control_input_ = dc_.q_;
    m_ci_.unlock();

    // std::cout << " Joint Control DC (NO MODE):  " << dc_.control_input_.transpose() << std::endl;
}