#include "gen3_controller/gen3_interface.h"


Gen3Interface::Gen3Interface(rclcpp::Node::SharedPtr node, DataContainer &dc): dc_(dc), node_(node)
{

    gen3_pos_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("/robot_data", 100);
    gen3_joint_cmd_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("/position_controller/commands", 100);
    gen3_status_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10, std::bind(&Gen3Interface::simStatusCallback, this, std::placeholders::_1));
    gen3_ft_sub_ = node_->create_subscription<geometry_msgs::msg::WrenchStamped>(
        "/ft_fit_data", 10, std::bind(&Gen3Interface::ft_StatusCallback, this, std::placeholders::_1));
    
}

Gen3Interface::~Gen3Interface()
{
}


void Gen3Interface::stateUpdate()
{
    rclcpp::Rate r(1000);
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node_);
        r.sleep();
    }
}

void Gen3Interface::ft_StatusCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
{
    if (is_first_ft)
    {
        dc_.ft_.resize(6);
        dc_.ft_.setZero();

        dc_.ftt_.resize(6);
        dc_.ftt_.setZero();
   
        dc_.ft_[0] = msg->wrench.force.y;
        dc_.ft_[1] = msg->wrench.force.x;
        dc_.ft_[2] = msg->wrench.force.z;
        dc_.ft_[2] = -dc_.ft_[2];
        dc_.ft_[3] = msg->wrench.torque.y;
        dc_.ft_[4] = msg->wrench.torque.x;
        dc_.ft_[5] = msg->wrench.torque.z;
        dc_.ft_[5] = -dc_.ft_[5];

        is_first_ft = false;
    }
    else
    {
        dc_.ft_[0] = msg->wrench.force.y;
        dc_.ft_[1] = msg->wrench.force.x;
        dc_.ft_[2] = msg->wrench.force.z;
        dc_.ft_[2] = -dc_.ft_[2];
        dc_.ft_[3] = msg->wrench.torque.y;
        dc_.ft_[4] = msg->wrench.torque.x;
        dc_.ft_[5] = msg->wrench.torque.z;
        dc_.ft_[5] = -dc_.ft_[5];

    }

    RCLCPP_INFO(node_->get_logger(), "Received ft data: force x: %f, force y: %f", msg->wrench.force.x, msg->wrench.force.y);
}


void Gen3Interface::simStatusCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    if (is_first_callback)
    {
        dc_.num_dof_ = 7;

        dc_.sim_timestamp_ = msg->header.stamp;
        dc_.sim_time_ = dc_.sim_timestamp_.seconds();

        dc_.q_.resize(dc_.num_dof_);
        dc_.q_dot_.resize(dc_.num_dof_);
        dc_.effort_.resize(dc_.num_dof_);
        dc_.q_.setZero();
        dc_.q_dot_.setZero();
        dc_.effort_.setZero();
        // dc_.ft_.resize(6);
        // dc_.ft_.setZero();
        dc_.g_.resize(dc_.num_dof_);
        dc_.g_.setZero();


        //joint_states 의 joint 순서가 다름  [joint1 joint2 joint4 joint5 joint3 joint6 joint7]

        for (int j=0; j<dc_.num_dof_; j++)
        {
            dc_.q_[j] = msg->position[j];
            dc_.q_dot_[j] = msg->velocity[j];
            dc_.effort_[j] = msg->effort[j];
        }
        dc_.q_[2] =msg->position[4];
        dc_.q_[3] =msg->position[2];
        dc_.q_[4] =msg->position[3];
        dc_.q_dot_[2] =msg->velocity[4];
        dc_.q_dot_[3] =msg->velocity[2];
        dc_.q_dot_[4] =msg->velocity[3];
        dc_.effort_[2] =msg->effort[4];
        dc_.effort_[3] =msg->effort[2];
        dc_.effort_[4] =msg->effort[3];
        

        dc_.control_input_.resize(dc_.num_dof_);
        dc_.control_input_.setZero();
        dc_.control_input_ << -0.0000001, 0.34934, -3.14156, -1.74539, 0.0, -1.04725, 1.57034;


        gen3_pos_msg_.position.resize(dc_.num_dof_);
        gen3_pos_msg_.velocity.resize(dc_.num_dof_);
        gen3_pos_msg_.effort.resize(dc_.num_dof_);

        gen3_joint_cmd_msg_.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
        gen3_joint_cmd_msg_.layout.dim[0].size = dc_.num_dof_;
        gen3_joint_cmd_msg_.layout.dim[0].stride = dc_.num_dof_;
        gen3_joint_cmd_msg_.layout.dim[0].label = "joints";

        is_first_callback = false;
        dc_.is_first_callback = is_first_callback;
    }
    else
    {
        for (int j=0; j<dc_.num_dof_; j++)
        {
            dc_.q_[j] = msg->position[j];
            dc_.q_dot_[j] = msg->velocity[j];
            dc_.effort_[j] = msg->effort[j];
        }

        dc_.q_[2] =msg->position[4];
        dc_.q_[3] =msg->position[2];
        dc_.q_[4] =msg->position[3];
        dc_.q_dot_[2] =msg->velocity[4];
        dc_.q_dot_[3] =msg->velocity[2];
        dc_.q_dot_[4] =msg->velocity[3];
        dc_.effort_[2] =msg->effort[4];
        dc_.effort_[3] =msg->effort[2];
        dc_.effort_[4] =msg->effort[3];


        dc_.sim_timestamp_ = msg->header.stamp;
        dc_.sim_time_ = dc_.sim_timestamp_.seconds();
    }
}

void Gen3Interface::pubCommand()
{
    rclcpp::Rate r(1000);
    while (rclcpp::ok())
    {
        if (!is_first_callback)
        {                

            for (int i = 0; i < 6; i++)
            {
                gen3_pos_msg_.position[i] = dc_.ftt_[i];
                gen3_pos_msg_.effort[i] = dc_.g_[i];
            }

            gen3_pos_msg_.header.stamp = node_->get_clock()->now();
            gen3_pos_pub_->publish(gen3_pos_msg_);
        }

        r.sleep();
    }
}

void Gen3Interface::gen3Command() 
{
    rclcpp::Rate r(1000);
    // std::vector<double> control_input_vector(dc_.num_dof_); // 크기를 dc_.num_dof_에 맞춰서 초기화

    while (rclcpp::ok()) 
    {
        if (!is_first_callback) 
        {
            std::vector<double> control_input_vector(dc_.control_input_.data(), dc_.control_input_.data() + dc_.control_input_.size());
            
            // std::copy(dc_.control_input_.begin(), dc_.control_input_.end(), control_input_vector.begin());
            gen3_joint_cmd_msg_.data = control_input_vector;

            gen3_joint_cmd_pub_->publish(gen3_joint_cmd_msg_);
        }
        r.sleep();
    }
}