#include <mutex>
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <fstream>

#include "gen3_controller/gen3_interface.h"

#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>

#include "gen3_controller/util.h"

#define DOF 7

# define MODE_INIT 105   // I
# define MODE_HOME 104   // H
# define MODE_ADMIT 102     // F
# define MODE_STOP 115      // S
# define MODE_1 113 //q
# define MODE_2 119 //w
# define MODE_3 101 //e
# define MODE_4 114 //r
# define MODE_5 116 //t
# define MODE_ADMIT_1 122 //z
# define MODE_ADMIT_2 120 //x
# define MODE_ADMIT_3 99 //c
// # define MODE_SCENE_4 118 //v

class Gen3Controller{
    public:
        Gen3Controller(rclcpp::Node::SharedPtr node, DataContainer &dc);
        ~Gen3Controller();
        void compute();
        void updateKinematicsDynamics();
        void computeControlInput();

    private:
        std::mutex m_dc_;
        std::mutex m_ci_;
        DataContainer &dc_;

        std::ofstream txt_right_;

        bool is_init_ = false;
        int mode_;

        double sim_time_ = 0.0;
        double control_start_time_ = 0.0;
        double mass_ob_ = 10.0;


        rclcpp::Time sim_timestamp_;  // 0초 0나노초


        // Robot State
        struct RobotState
        {

            Eigen::VectorXd q_;
            Eigen::VectorXd q_init_;
            Eigen::VectorXd q_dot_;
            Eigen::VectorXd q_dot_zero_;
            Eigen::VectorXd q_desired;
            Eigen::VectorXd effort_;
            Eigen::Vector6d ft_;
            Eigen::Vector6d ft_before_;
            Eigen::Vector6d ft_fix_;

            Eigen::Vector3d x_desired_;
            Eigen::Vector3d xa_desired;

            // Kinematics & Dynamics
            RigidBodyDynamics::Model robot_model;
            RigidBodyDynamics::Model &robot_model_ = robot_model;

            Eigen::MatrixXd A_;
            Eigen::VectorXd g_;

            Eigen::Vector6d x_dot_;
            Eigen::Vector6d x_dot_before_;
            Eigen::VectorXd x_angle_;

            Eigen::Isometry3d x_;
            Eigen::Isometry3d x_init_;
            Eigen::MatrixXd j_temp_;
            Eigen::MatrixXd j_;

            Eigen::VectorXd control_input_;
            Eigen::VectorXd control_assist_;
        };
        RobotState right_arm_;
};