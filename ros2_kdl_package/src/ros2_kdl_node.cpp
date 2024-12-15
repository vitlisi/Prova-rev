// Copyright (C) 2007  Francois Cauwe <francois at cauwe dot org>
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.

#include <stdio.h>
#include <iostream>
#include <chrono>
#include <cstdlib>
#include <memory>

#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"

#include "kdl_robot.h"
#include "kdl_control.h"
#include "kdl_planner.h"
#include "kdl_parser/kdl_parser.hpp"

using namespace KDL;
using FloatArray = std_msgs::msg::Float64MultiArray;
using namespace std::chrono_literals;

class Iiwa_pub_sub : public rclcpp::Node
{
public:
    Iiwa_pub_sub(int trajectory_index)
        : Node("ros2_kdl_node"),
          node_handle_(std::shared_ptr<Iiwa_pub_sub>(this)),
          trajectory_index_(trajectory_index),
          posizione_iniziale_raggiunta_(false),
          soglia_errore_(0.2)
    {
        // Declare cmd_interface parameter (position, velocity, or effort)
        declare_parameter("cmd_interface", "position"); // defaults to "position"
        get_parameter("cmd_interface", cmd_interface_);
        RCLCPP_INFO(get_logger(), "Current cmd interface is: '%s'", cmd_interface_.c_str());

        if (!(cmd_interface_ == "position" || cmd_interface_ == "velocity" || cmd_interface_ == "effort"))
        {
            RCLCPP_ERROR(get_logger(), "Selected cmd interface is not valid! Use 'position', 'velocity' or 'effort' instead...");
            return;
        }

        iteration_ = 0;
        t_ = 0;
        joint_state_available_ = false;

        // Retrieve the robot_description parameter
        auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node_handle_, "robot_state_publisher");
        while (!parameters_client->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                rclcpp::shutdown();
            }
            RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
        }
        auto parameter = parameters_client->get_parameters({"robot_description"});

        // Create KDLRobot structure
        KDL::Tree robot_tree;
        if (!kdl_parser::treeFromString(parameter[0].value_to_string(), robot_tree))
        {
            std::cout << "Failed to retrieve robot_description param!";
        }
        robot_ = std::make_shared<KDLRobot>(robot_tree);

        // Create joint arrays
        unsigned int nj = robot_->getNrJnts();
        KDL::JntArray q_min(nj), q_max(nj);
        q_min.data << -2.96, -2.09, -2.96, -2.09, -2.96, -2.09, -2.96; // TODO: read from URDF file
        q_max.data << 2.96, 2.09, 2.96, 2.09, 2.96, 2.09, 2.96;         // TODO: read from URDF file
        robot_->setJntLimits(q_min, q_max);
        joint_positions_.resize(nj);
        joint_velocities_.resize(nj);
        joint_positions_cmd_.resize(nj);
        joint_velocities_cmd_.resize(nj);
        joint_efforts_cmd_.resize(nj);
        joint_efforts_.resize(nj);
        joint_efforts_.data.setZero();

        // Subscriber to joint states
        jointSubscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&Iiwa_pub_sub::joint_state_subscriber, this, std::placeholders::_1));

        // Wait for the joint_state topic
        while (!joint_state_available_)
        {
            RCLCPP_INFO(this->get_logger(), "No data received yet! ...");
            rclcpp::spin_some(node_handle_);
        }

        // Define the initial joint positions (desired positions)
        init_joint_positions_.resize(nj);
        init_joint_positions_(0) = 0.5;     // Joint 1
        init_joint_positions_(1) = -0.7854; // Joint 2
        init_joint_positions_(2) = 0.0;     // Joint 3
        init_joint_positions_(3) = 1.3962;  // Joint 4
        init_joint_positions_(4) = 0.0;     // Joint 5
        init_joint_positions_(5) = 0.6109;  // Joint 6
        init_joint_positions_(6) = 0.0;     // Joint 7

        // Update KDLRobot object with initial joint positions
        robot_->update(toStdVector(init_joint_positions_.data), std::vector<double>(nj, 0.0));

        // Add end-effector frame if necessary
        KDL::Frame f_T_ee = KDL::Frame::Identity();
        robot_->addEE(f_T_ee);

        // Compute EE frame based on initial joint positions
        init_cart_pose_ = robot_->getEEFrame();

        // Initialize controller with the robot model
        controller_ = std::make_shared<KDLController>(*robot_);

        // EE's trajectory initial position
        Eigen::Vector3d init_position(init_cart_pose_.p.data);

        // EE's trajectory end position (just opposite y)
        Eigen::Vector3d end_position;
        end_position << init_position[0], -init_position[1], init_position[2];

        // Plan trajectory based on trajectory_index_
        double traj_duration = 1.5, acc_duration = 0.375, trajRadius = 0.1;

        // Create planners
        planners_.emplace_back(traj_duration, acc_duration, init_position, end_position); // Linear Trapezoidal
        planners_.back().setProfileType(ProfileType::TRAPEZOIDAL);
        planners_.back().setTrajectoryType(TrajectoryType::LINEAR);

        planners_.emplace_back(traj_duration, acc_duration, init_position, end_position); // Linear Cubic
        planners_.back().setProfileType(ProfileType::CUBIC);
        planners_.back().setTrajectoryType(TrajectoryType::LINEAR);

        planners_.emplace_back(traj_duration, init_position, trajRadius); // Circular Trapezoidal
        planners_.back().setProfileType(ProfileType::TRAPEZOIDAL);
        planners_.back().setTrajectoryType(TrajectoryType::CIRCULAR);

        planners_.emplace_back(traj_duration, init_position, trajRadius); // Circular Cubic
        planners_.back().setProfileType(ProfileType::CUBIC);
        planners_.back().setTrajectoryType(TrajectoryType::CIRCULAR);

        // Select the planner based on trajectory_index_
        if (trajectory_index_ >= 0 && trajectory_index_ < static_cast<int>(planners_.size()))
        {
            planner_ = planners_[trajectory_index_];
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Invalid trajectory index provided. Please provide a trajectory index between 0 and %ld.", planners_.size() - 1);
            return;
        }

        // Create cmd publisher based on cmd_interface_
        if (cmd_interface_ == "position")
        {
            cmdPublisher_ = this->create_publisher<FloatArray>("/iiwa_arm_controller/commands", 10);
        }
        else if (cmd_interface_ == "velocity")
        {
            cmdPublisher_ = this->create_publisher<FloatArray>("/velocity_controller/commands", 10);
        }
        else if (cmd_interface_ == "effort")
        {
            cmdPublisher_ = this->create_publisher<FloatArray>("/effort_controller/commands", 10);
        }

        // Initialize desired commands with initial joint positions
        for (long int i = 0; i < init_joint_positions_.rows(); ++i)
        {
            desired_commands_[i] = init_joint_positions_(i);
        }

        // Create msg and publish initial commands
        std_msgs::msg::Float64MultiArray cmd_msg;
        cmd_msg.data = desired_commands_;
        cmdPublisher_->publish(cmd_msg);

        // Start a single timer for the entire logic (initialization + trajectory)
        timer_ = this->create_wall_timer(std::chrono::milliseconds(10),
                                         std::bind(&Iiwa_pub_sub::cmd_publisher, this));

        RCLCPP_INFO(this->get_logger(), "Starting control loop ...");
    }

private:
    void cmd_publisher()
    {
        // If we are in effort mode and have not yet reached the initial position,
        // apply control torques to reach that position.
        if (cmd_interface_ == "effort" && !posizione_iniziale_raggiunta_)
        {
            // Compute the joint error between desired and current position
            KDL::JntArray q_error(init_joint_positions_.rows());
            for (unsigned int i = 0; i < init_joint_positions_.rows(); ++i) {
                q_error(i) = init_joint_positions_(i) - joint_positions_(i);
            }

            // Control gains for the initialization phase
            double Kp = 100.0; // Proportional gain
            double Kd = 20.0;  // Derivative gain

            // Compute control torques
            Eigen::VectorXd torques = Kp * q_error.data - Kd * joint_velocities_.data;

            // Gravity compensation
            robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));
            Eigen::VectorXd gravity = robot_->getGravity();
            torques += gravity;

            // Assign computed torques to joint effort commands
            for (int i = 0; i < torques.size(); ++i)
            {
                joint_efforts_cmd_(i) = torques(i);
                desired_commands_[i] = joint_efforts_cmd_(i);
            }

            // Create message and publish
            std_msgs::msg::Float64MultiArray cmd_msg;
            cmd_msg.data = desired_commands_;
            cmdPublisher_->publish(cmd_msg);

            // Check if the initial position has been reached
            if (q_error.data.norm() <= soglia_errore_)
            {
                posizione_iniziale_raggiunta_ = true;
                RCLCPP_INFO(this->get_logger(), "Initial position reached.");
                RCLCPP_INFO(this->get_logger(), "Starting trajectory execution ...");
            }

            // Return without executing the trajectory until the initial position is reached
            return;
        }

        // From here on, we are either in "position"/"velocity" mode
        // or we have already reached the initial position in "effort" mode.

        iteration_ = iteration_ + 1;

        // Define trajectory
        double total_time = 1.5;
        int trajectory_len = 150;
        int loop_rate = trajectory_len / total_time;
        double dt = 1.0 / loop_rate;
        t_ += dt;

        // Update the KDLRobot structure with current states
        robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));

        if (t_ <= total_time)
        {
            // Retrieve the trajectory point
            trajectory_point p = planner_.compute_trajectory(t_);

            // Compute EE frame
            KDL::Frame cartpos = robot_->getEEFrame();

            // Compute desired Frame
            KDL::Frame desFrame;
            desFrame.M = cartpos.M;
            desFrame.p = toKDL(p.pos);

            // Compute errors
            Eigen::Vector3d error = computeLinearError(p.pos, Eigen::Vector3d(cartpos.p.data));
            Eigen::Vector3d o_error = computeOrientationError(toEigen(init_cart_pose_.M), toEigen(cartpos.M));
            
            //print only in position or velocity mode
            if (cmd_interface_ == "position" && cmd_interface_ == "velocity") {
            	std::cout << "The error norm is : " << error.norm() << std::endl;
            	std::cout << "The error_orientation is : " << o_error.norm() << std::endl;
            	}

            if (cmd_interface_ == "position")
            {
                // Next Frame
                KDL::Frame nextFrame;
                nextFrame.M = cartpos.M;
                nextFrame.p = cartpos.p + (toKDL(p.vel) + toKDL(1 * error)) * dt;

                // Compute IK
                joint_positions_cmd_ = joint_positions_;
                robot_->getInverseKinematics(nextFrame, joint_positions_cmd_);
            }
            else if (cmd_interface_ == "velocity")
            {
                // Compute differential IK
                Vector6d cartvel;
                cartvel << p.vel + 5 * error, o_error;
                joint_velocities_cmd_.data = pseudoinverse(robot_->getEEJacobian().data) * cartvel;
            }
            else if (cmd_interface_ == "effort")
            {
                // Effort control using idCntr
                // Desired position
                KDL::Frame desPos;
                desPos.p = toKDL(p.pos);
                desPos.M = cartpos.M;

                // Desired velocity
                KDL::Twist desVel;
                desVel.vel = toKDL(p.vel);
                desVel.rot = KDL::Vector(0.0, 0.0, 0.0); // Zero angular velocity

                // Desired acceleration
                KDL::Twist desAcc;
                desAcc.vel = toKDL(p.acc);
                desAcc.rot = KDL::Vector(0.0, 0.0, 0.0); // Zero angular acceleration

                // Control gains
                double Kpp = 350;
                double Kpo = 350;
                double Kdp = 100;
                double Kdo = 100;

                // Compute torques using idCntr
                Eigen::VectorXd torques = controller_->idCntr(desPos, desVel, desAcc, Kpp, Kpo, Kdp, Kdo);

                // Assign computed torques to joint effort commands
                for (int i = 0; i < torques.size(); ++i)
                {
                    joint_efforts_cmd_(i) = torques(i);
                    RCLCPP_INFO(this->get_logger(), "q_dot[%u]: %.6f", i, torques(i));
                }
            }

            // Update KDLRobot structure
            robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));

            if (cmd_interface_ == "position")
            {
                // Set joint position commands
                for (long int i = 0; i < joint_positions_.data.size(); ++i)
                {
                    desired_commands_[i] = joint_positions_cmd_(i);
                }
            }
            else if (cmd_interface_ == "velocity")
            {
                // Set joint velocity commands
                for (long int i = 0; i < joint_velocities_.data.size(); ++i)
                {
                    desired_commands_[i] = joint_velocities_cmd_(i);
                }
            }
            else if (cmd_interface_ == "effort")
            {
                // Set joint effort commands
                for (long int i = 0; i < joint_efforts_.data.size(); ++i)
                {
                    desired_commands_[i] = joint_efforts_cmd_(i);
                }
            }

            // Create message and publish
            std_msgs::msg::Float64MultiArray cmd_msg;
            cmd_msg.data = desired_commands_;
            cmdPublisher_->publish(cmd_msg);
        }
        else
        {
            RCLCPP_INFO_ONCE(this->get_logger(), "Trajectory executed successfully ...");

            robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));

            // Get the current pose of the manipulator
            KDL::Frame current_cart_pose = robot_->getEEFrame();

            if (cmd_interface_ == "velocity")
            {
                // For velocity control, send zero velocities to stop the robot
                for (long int i = 0; i < desired_commands_.size(); ++i)
                {
                    desired_commands_[i] = 0.0;
                }

                // Create message and publish
                std_msgs::msg::Float64MultiArray cmd_msg;
                cmd_msg.data = desired_commands_;
                cmdPublisher_->publish(cmd_msg);
            }
            else if (cmd_interface_ == "effort")
            {
                // Continue using idCntr to maintain the final position
                // Desired position
                KDL::Frame desPos = current_cart_pose;

                // Desired velocities and accelerations set to zero
                KDL::Twist desVel;
                desVel.vel = KDL::Vector(0.0, 0.0, 0.0);
                desVel.rot = KDL::Vector(0.0, 0.0, 0.0);

                KDL::Twist desAcc;
                desAcc.vel = KDL::Vector(0.0, 0.0, 0.0);
                desAcc.rot = KDL::Vector(0.0, 0.0, 0.0);

                // Control gains for final position maintenance
                double Kpp = 50;
                double Kpo = 50;
                double Kdp = 5;
                double Kdo = 50;

                // Compute torques using idCntr
                Eigen::VectorXd torques = controller_->idCntr(desPos, desVel, desAcc, Kpp, Kpo, Kdp, Kdo);

                // Assign computed torques to joint effort commands
                for (int i = 0; i < torques.size(); ++i)
                {
                    joint_efforts_cmd_(i) = torques(i);
                    desired_commands_[i] = joint_efforts_cmd_(i);
                }

                // Create message and publish
                std_msgs::msg::Float64MultiArray cmd_msg;
                cmd_msg.data = desired_commands_;
                cmdPublisher_->publish(cmd_msg);
            }
            // For position control, the robot maintains its last commanded position automatically.
        }
    }

    void joint_state_subscriber(const sensor_msgs::msg::JointState &sensor_msg)
    {
        joint_state_available_ = true;
        for (unsigned int i = 0; i < sensor_msg.position.size(); i++)
        {
            joint_positions_.data[i] = sensor_msg.position[i];
            joint_velocities_.data[i] = sensor_msg.velocity[i];
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointSubscriber_;
    rclcpp::Publisher<FloatArray>::SharedPtr cmdPublisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Node::SharedPtr node_handle_;

    std::vector<double> desired_commands_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    KDL::JntArray joint_positions_;
    KDL::JntArray joint_velocities_;

    KDL::JntArray joint_positions_cmd_;
    KDL::JntArray joint_velocities_cmd_;
    KDL::JntArray joint_efforts_cmd_;
    KDL::JntArray joint_efforts_;

    std::shared_ptr<KDLRobot> robot_;
    KDLPlanner planner_;
    std::vector<KDLPlanner> planners_;
    std::shared_ptr<KDLController> controller_;

    int iteration_;
    bool joint_state_available_;
    double t_;
    std::string cmd_interface_;
    KDL::Frame init_cart_pose_;
    KDL::JntArray init_joint_positions_;
    int trajectory_index_;
    bool posizione_iniziale_raggiunta_;
    double soglia_errore_; // Threshold to determine if the initial position has been reached
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // Parse command-line arguments
    int trajectory_index = 0; // Default value
    if (argc > 1)
    {
        trajectory_index = std::atoi(argv[1]);
    }
    else
    {
        std::cout << "Please provide a trajectory index (0-3)." << std::endl;
        return 1;
    }

    auto node = std::make_shared<Iiwa_pub_sub>(trajectory_index);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

