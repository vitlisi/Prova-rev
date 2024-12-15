// Copyright (C) 2007 Francois Cauwe
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
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"

#include "kdl_robot.h"
#include "kdl_control.h"
#include "kdl_planner.h"
#include "utils.h"
#include "kdl_parser/kdl_parser.hpp"

#include <Eigen/Geometry> // For Eigen::Quaterniond

using namespace KDL;
using FloatArray = std_msgs::msg::Float64MultiArray;
using namespace std::chrono_literals;

class Iiwa_pub_sub : public rclcpp::Node
{
public:
    Iiwa_pub_sub()
        : Node("ros2_kdl_vision_control")
    {
        // Declare the parameter cmd_interface (position, velocity, effort)
        declare_parameter("cmd_interface", "position"); // Default is "position"
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
        aruco_pose_available_ = false;
        posizione_iniziale_raggiunta_ = false;
        soglia_errore_ = 0.05;

        // Declare the parameter "task"
        declare_parameter("task", "niente"); // Default is "niente"
        get_parameter("task", task_);
        RCLCPP_INFO(get_logger(), "Current task is: '%s'", task_.c_str());

        // Validate the task parameter
        if (!(task_ == "positioning" || task_ == "look-at-point") && cmd_interface_ == "velocity")
        {
            RCLCPP_ERROR(get_logger(), "Invalid task selected! Use 'positioning' or 'look-at-point' instead.");
            return;
        }

        // Declare the parameter "trajectory"
        declare_parameter("trajectory", "linear"); // Default is "linear"
        get_parameter("trajectory", trajectory_);
        RCLCPP_INFO(get_logger(), "Current trajectory is: '%s'", trajectory_.c_str());

        // Validate the trajectory parameter
        if (!(trajectory_ == "linear" || trajectory_ == "circular"))
        {
            RCLCPP_ERROR(get_logger(), "Invalid trajectory selected! Use 'linear' or 'circular' instead.");
            return;
        }

        // Retrieve the robot_description parameter
        auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this, "robot_state_publisher");
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

        // Create the KDLRobot structure
        KDL::Tree robot_tree;
        if (!kdl_parser::treeFromString(parameter[0].value_to_string(), robot_tree))
        {
            std::cout << "Failed to retrieve robot_description param!";
        }
        robot_ = std::make_shared<KDLRobot>(robot_tree);

        // Create the joint arrays
        unsigned int nj = robot_->getNrJnts();
        KDL::JntArray q_min(nj), q_max(nj);
        q_min.data << -2.96, -2.09, -2.96, -2.09, -2.96, -2.09, -2.96; // TODO: Read from URDF file
        q_max.data << 2.96, 2.09, 2.96, 2.09, 2.96, 2.09, 2.96;         // TODO: Read from URDF file
        robot_->setJntLimits(q_min, q_max);
        joint_positions_.resize(nj);
        joint_velocities_.resize(nj);
        joint_positions_cmd_.resize(nj);
        joint_velocities_cmd_.resize(nj);
        joint_efforts_cmd_.resize(nj);
        joint_efforts_cmd_.data.setZero();

        // Subscribe to joint states
        jointSubscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&Iiwa_pub_sub::joint_state_subscriber, this, std::placeholders::_1));

        // Subscribe to the ArUco marker pose
        aruco_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/aruco_pose", 10, std::bind(&Iiwa_pub_sub::aruco_pose_subscriber, this, std::placeholders::_1));

        // Wait for the joint_states topic
        while (!joint_state_available_)
        {
            RCLCPP_INFO(this->get_logger(), "No data received yet! ...");
            rclcpp::spin_some(this->get_node_base_interface());
        }

        // Define the initial joint positions (desired positions, for effort_control)
        init_joint_positions_.resize(nj);
        init_joint_positions_(0) = 1;      // Joint 1
        init_joint_positions_(1) = -0.5;  // Joint 2
        init_joint_positions_(2) = 0.3;   // Joint 3
        init_joint_positions_(3) = 1.75;  // Joint 4
        init_joint_positions_(4) = 1;     // Joint 5
        init_joint_positions_(5) = 1.4;   // Joint 6
        init_joint_positions_(6) = -0.8;  // Joint 7

        // Update the KDLRobot object
        robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));
        KDL::Frame f_T_ee = KDL::Frame::Identity();
        robot_->addEE(f_T_ee);
        robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));

        // Calculate the EE frame
        init_cart_pose_ = robot_->getEEFrame();

        // Compute inverse kinematics
        KDL::JntArray q(nj);
        robot_->getInverseKinematics(init_cart_pose_, q);

        // Initialize the controller
        controller_ = std::make_shared<KDLController>(*robot_);

        // Initial trajectory EE position (based on task)
        Eigen::Vector3d init_position;

        // Set init_position based on the task
        if (cmd_interface_ == "effort")
        {
            // Update the robot with initial joint positions
            robot_->update(toStdVector(init_joint_positions_.data), std::vector<double>(7, 0.0)); // Zero velocity
            KDL::Frame init_cart_pose = robot_->getEEFrame(); // Get initial Cartesian pose
            init_position = Eigen::Vector3d(init_cart_pose.p.x(), init_cart_pose.p.y(), init_cart_pose.p.z());
        }
        else
        {
            // Use a default offset position
            init_position = Eigen::Vector3d(init_cart_pose_.p.data) - Eigen::Vector3d(0, 0, 0.1);
        }

        // Final trajectory EE position (dynamic)
        Eigen::Vector3d end_position;

        // Set end_position based on the task
        if (task_ == "positioning")
        {
            end_position << 0.2, -0.5, 0.5;
            end_position[0] += 0.5; // Add an offset of +0.5 in x
        }
        else
        {
            end_position << init_position[0], -init_position[1], init_position[2];
        }

        // Define initial orientation (assumed identity)
        Eigen::Quaterniond orientationInit = Eigen::Quaterniond::Identity();

        // Define final orientation (tag orientation in RPY)
        double roll = 0.0;
        double pitch = -2.2; // Offset
        double yaw = 0.0;

        Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

        Eigen::Quaterniond orientationEnd = yawAngle * pitchAngle * rollAngle;

        // Plan the trajectory
        double traj_duration = 12;
        double acc_duration = 3;
        double t = 0.0;
        double trajRadius = 0.2;

        if (trajectory_ == "linear")
        {
            planner_ = KDLPlanner(traj_duration, acc_duration, init_position, end_position, orientationInit, orientationEnd);
        }
        else
        {
            planner_ = KDLPlanner(traj_duration, acc_duration, init_position, trajRadius);
        }

        // Retrieve the first trajectory point
        trajectory_point p = planner_.compute_trajectory(t);

        // Compute errors
        Eigen::Vector3d pos_error = computeLinearError(p.pos, Eigen::Vector3d(init_cart_pose_.p.data));

        // Add an attribute for the initial position
        robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));
        Eigen::VectorXd q_init_; // Initial joint configuration

        // Create the publisher for commands based on the interface
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

        // Create the timer for the control loop
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&Iiwa_pub_sub::cmd_publisher, this));

        // Initialize the desired commands
        desired_commands_.resize(nj, 0.0);
    }

private:
    void cmd_publisher()
    {
        iteration_ += 1;

        if (task_ == "positioning" && cmd_interface_ == "velocity")
        {
            // Call the function to compute positioning
            compute_positioning();      
        }
        else if (task_ == "look-at-point" && cmd_interface_ == "velocity")
        {
            // Call the function to compute the control law
            compute_look_at_point_control();
        }
        else if (cmd_interface_ == "effort") 
        {
            if (!posizione_iniziale_raggiunta_)
            {
                // Pre-trajectory
                return_to_initial_position();
            }
            else
            {
                // Execute trajectory
                effort_control();
            }
        }
    }

    void joint_state_subscriber(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        joint_state_available_ = true;
        for (unsigned int i = 0; i < msg->position.size(); i++)
        {
            joint_positions_.data[i] = msg->position[i];
            joint_velocities_.data[i] = msg->velocity[i];
        }
    }

    void aruco_pose_subscriber(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        // Store the pose of the ArUco marker
        aruco_pose_ = KDL::Frame(
            KDL::Rotation::Quaternion(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w),
            KDL::Vector(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z)
        );

        // Update a variable indicating that the pose is available
        aruco_pose_available_ = true;
        q_init_ = robot_->getJntValues(); // Dimension (nj)

        // Store the position of the object relative to the camera
        Po_w_ = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        
        //RCLCPP_INFO(this->get_logger(), "ArUco Position: [x: %.3f, y: %.3f, z: %.3f]", Po_w_.x(), Po_w_.y(), Po_w_.z());
        //RCLCPP_INFO(this->get_logger(), "ArUco pose updated");
     }
     
    void compute_positioning()
    {
        // Define trajectory
        double total_time = 12;  
        int trajectory_len = 150; 
        int loop_rate = trajectory_len / total_time;
        double dt = 1.0 / loop_rate;

        if (t_ < total_time)
        {
            t_ += dt;
            // Retrieve the trajectory point
            trajectory_point p = planner_.compute_trajectory(t_);

            // Compute the EE frame
            KDL::Frame cartpos = robot_->getEEFrame();

            // Compute the desired frame
            KDL::Frame desFrame;
            desFrame.M = KDL::Rotation::Quaternion(
                p.orientation.x(), p.orientation.y(), p.orientation.z(), p.orientation.w());
            desFrame.p = toKDL(p.pos);

            // Compute errors
            Eigen::Vector3d pos_error = computeLinearError(toEigen(desFrame.p), toEigen(cartpos.p));
            Eigen::Vector3d ori_error = computeOrientationError(toEigen(desFrame.M), toEigen(cartpos.M));
            //std::cout << "The error norm is: " << pos_error.norm() << std::endl;

            // Combine errors for control
            Vector6d total_error;
            total_error.head<3>() = pos_error;
            total_error.tail<3>() = ori_error;

            // Control gains (adjust these values appropriately)
            double Kp_pos = 5.0;
            double Kp_ori = 2.0;

            // Use Jacobian to compute joint velocities to reduce error
            Vector6d cart_vel;
            cart_vel.head<3>() = p.vel + Kp_pos * pos_error;
            cart_vel.tail<3>() = Kp_ori * ori_error;
            joint_velocities_cmd_.data = pseudoinverse(robot_->getEEJacobian().data) * cart_vel;

            for (size_t i = 0; i < joint_velocities_cmd_.rows(); ++i)
            {
                desired_commands_[i] = joint_velocities_cmd_(i);
            }

            // Update the KDL robot structure
            robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));

            // Create the message and publish
            std_msgs::msg::Float64MultiArray cmd_msg;
            cmd_msg.data = desired_commands_;
            cmdPublisher_->publish(cmd_msg);
        }
        else
        {
            RCLCPP_INFO_ONCE(this->get_logger(), "Trajectory executed successfully ...");

            // Send joint velocity commands to zero
            for (size_t i = 0; i < joint_velocities_.rows(); ++i)
            {
                desired_commands_[i] = 0.0;
            }

            // Create the message and publish
            std_msgs::msg::Float64MultiArray cmd_msg;
            cmd_msg.data = desired_commands_;
            cmdPublisher_->publish(cmd_msg);
        }
    }

    void compute_look_at_point_control()
    {
        if (!aruco_pose_available_)
        {
            RCLCPP_WARN(this->get_logger(), "ArUco pose not available.");
            return;
        }

        // Update the KDL robot structure
        robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));

        // Get the current pose of the camera (end-effector)
        KDL::Frame T_c_w = robot_->getEEFrame(); // Current pose of the end-effector in the world frame

        Eigen::Vector3d Pc_w = toEigen(T_c_w.p); // Camera position in the world frame (located on the EE)

        // Orientation of the EE is different from the camera's; calculate camera orientation
        Eigen::Matrix3d R_ee_to_camera;
        R_ee_to_camera << 
            0,  0, -1,
            0, -1,  0,
           -1,  0,  0;

        Eigen::Matrix3d R_ee = toEigen(T_c_w.M);
        Eigen::Matrix3d Rc = R_ee * R_ee_to_camera;

        // Calculate the object's position in the camera frame
        Eigen::Vector3d Po_c = Po_w_;

        // Transform the marker position into the world frame
        Eigen::Vector3d Po_w = Rc * Po_c + Pc_w;

        // Calculate the normalized look-at vector
        Eigen::Vector3d s = (Po_w - Pc_w).normalized();

        Eigen::Vector3d z_des;
        if (trajectory_ == "circular")
        {
            // For circular trajectory
            z_des = -s; // Look towards the center
        }
        else
        {
            // For linear trajectory
            if ((Pc_w - Po_w_).y() > 0)
            {
                z_des = -s; // If the manipulator is on the positive y side
            }
            else
            {
                z_des = s; // If the manipulator is on the negative y side
            }
        }

        Eigen::Vector3d x_tmp(1.0, 0.0, 0.0); // Arbitrary reference, must be different from s

        // Change reference if s is parallel to x_tmp
        if (std::abs(s.dot(x_tmp)) > 0.99)
        {
            x_tmp = Eigen::Vector3d(0.0, 1.0, 0.0);
        }

        // Calculate x_des and y_des orthogonal to each other
        Eigen::Vector3d x_des = x_tmp.cross(z_des).normalized();
        Eigen::Vector3d y_des = z_des.cross(x_des).normalized();

        // Construct the desired rotation matrix
        Eigen::Matrix3d R_des;
        R_des.col(0) = x_des;
        R_des.col(1) = y_des;
        R_des.col(2) = z_des;

        // Adapt the orientation of the end-effector (camera)
        //Eigen::Matrix3d R_des = Eigen::Matrix3d::Identity();
        R_ee_des = R_des * R_ee_to_camera.transpose();

        // Compute S(s), the antisymmetric operator
        Eigen::Matrix3d S_s;
        S_s <<     0, -s(2),  s(1),
                 s(2),     0, -s(0),
                -s(1),  s(0),     0;

        // Compute L(s), the mapping matrix
        Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
        Eigen::Matrix<double, 3, 6> Ls;
        Ls.block<3,3>(0,0) = - (1.0 / Po_c.norm()) * (I - s * s.transpose());
        Ls.block<3,3>(0,3) = S_s;

        // Compute R
        Eigen::Matrix<double, 6, 6> R;
        R.setZero();
        R.block<3,3>(0,0) = Rc;
        R.block<3,3>(3,3) = Rc;

        // Compute L
        Eigen::Matrix<double, 3, 6> L = Ls * R;

        // Compute J_c
        Eigen::MatrixXd Jc = robot_->getEEJacobian().data;

        // Compute L J_c
        Eigen::MatrixXd LJc = L * Jc;

        // Compute the error e
        Eigen::Vector3d sd(0.0, 0.0, 1.0); // Desired vector (camera's z-axis)
        Eigen::Vector3d e = sd.cross(s);
        double error_norm = e.norm();

        // Compute (L J_c)^dagger
        Eigen::MatrixXd LJc_pinv = pseudoinverse(LJc);

        // Compute N
        Eigen::MatrixXd N = Eigen::MatrixXd::Identity(robot_->getNrJnts(), robot_->getNrJnts()) - LJc_pinv * LJc;

        double k_0 = 5;
        Eigen::VectorXd q_current = robot_->getJntValues();
        Eigen::VectorXd q_dot_0 = -k_0 * (q_current - q_init_);

        double k = 2; // Proportional gain
        Eigen::VectorXd q_dot = k * LJc_pinv * e + N * q_dot_0;

        double max_speed = 0.5; // Limit joint speeds
        for (size_t i = 0; i < q_dot.size(); ++i)
        {
            if (q_dot(i) > max_speed) q_dot(i) = max_speed;
            else if (q_dot(i) < -max_speed) q_dot(i) = -max_speed;
        }

        if (cmd_interface_ == "velocity")
        {
            for (size_t i = 0; i < q_dot.size(); ++i)
            {
                desired_commands_[i] = q_dot(i);
                RCLCPP_INFO(this->get_logger(), "q_dot[%zu]: %.6f", i, q_dot(i));
            }
        }

        std_msgs::msg::Float64MultiArray cmd_msg;
        cmd_msg.data = desired_commands_;
        cmdPublisher_->publish(cmd_msg);
        
        /*Debug
        RCLCPP_INFO(this->get_logger(), "Look-At Vector (s): [x: %.3f, y: %.3f, z: %.3f]", s.x(), s.y(), s.z());
        RCLCPP_INFO(this->get_logger(), "Error Vector: [x: %.3f, y: %.3f, z: %.3f]", e.x(), e.y(), e.z());
        RCLCPP_INFO(this->get_logger(), "Error Norm: %.6f", error_norm); */
    }

    void return_to_initial_position() {
    	// Calculate the joint error between the desired and current positions
    	KDL::JntArray q_error(init_joint_positions_.rows());
  	  for (unsigned int i = 0; i < init_joint_positions_.rows(); ++i) {
 	       q_error(i) = init_joint_positions_(i) - joint_positions_(i);
 	   }

 	// Control gains
  	double Kp = 150.0; // Proportional gain
   	double Kd = 20.0;  // Derivative gain

   	// Calculate control torques
  	Eigen::VectorXd torques = Kp * q_error.data - Kd * joint_velocities_.data;

 	// Gravity compensation
  	robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));
   	Eigen::VectorXd gravity = robot_->getGravity();
   	torques += gravity;

  	// Assign calculated torques to joint effort commands
  	  for (int i = 0; i < torques.size(); ++i) {
  	      joint_efforts_cmd_(i) = torques(i);
  	      desired_commands_[i] = joint_efforts_cmd_(i);
 	   }

 	// Create and publish the message
   	std_msgs::msg::Float64MultiArray cmd_msg;
  	cmd_msg.data = desired_commands_;
   	cmdPublisher_->publish(cmd_msg);
  	RCLCPP_INFO_ONCE(this->get_logger(), "Returning manipulator to initial position...");

  	// Check if the initial position has been reached
  	  if (q_error.data.norm() <= soglia_errore_) {
  	      posizione_iniziale_raggiunta_ = true;
  	      RCLCPP_INFO(this->get_logger(), "Initial position reached.");
   	     RCLCPP_INFO(this->get_logger(), "Starting trajectory execution...");
  	  }
     }

     void effort_control() {
    	// Define trajectory
   	double total_time = 12;
   	int trajectory_len = 150;
   	int loop_rate = trajectory_len / total_time;
  	double dt = 1.0 / loop_rate;
  	t_ += dt;

   	// Update the KDLRobot structure with current states
  	robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));

  	double Kpp = 550;  // Proportional position gain
  	double Kpo = 750;  // Proportional orientation gain
  	double Kdp = 200;  // Derivative position gain
  	double Kdo = 200;  // Derivative orientation gain

   	 if (t_ <= total_time) {
   	     RCLCPP_INFO(this->get_logger(), "Executing trajectory...");
   	     maintenance_message_printed = false; // Reset flag during trajectory execution

   	     // Retrieve the trajectory point
    	    trajectory_point p = planner_.compute_trajectory(t_);

    	    // Compute EE frame
    	    KDL::Frame cartpos = robot_->getEEFrame();

    	    // Retrieve orientation error
    	    compute_look_at_point_control();

    	    // Calculate orientation error between R_ee_des and the current orientation
    	    // Eigen::Vector3d ori_error = computeOrientationError(R_ee_des, toEigen(cartpos.M));
    	    // RCLCPP_INFO(this->get_logger(), "Orientation error norm: %.6f", ori_error.norm());

     	   // Effort control using idCntr
    	    KDL::Frame desPos;
    	    desPos.p = toKDL(p.pos);

     	   // Create desPos.M using quaternions
     	   Eigen::Quaterniond quat(R_ee_des);
     	   desPos.M = KDL::Rotation::Quaternion(quat.x(), quat.y(), quat.z(), quat.w());

     	   Eigen::Vector3d pos_error = toEigen(cartpos.p) - p.pos;

     	   // Desired velocity
     	   KDL::Twist desVel;
     	   desVel.vel = toKDL(p.vel);
    	   desVel.rot = KDL::Vector(0.0, 0.0, 0.0);

     	   // Desired acceleration
    	   KDL::Twist desAcc;
     	   desAcc.vel = toKDL(p.acc);
    	   desAcc.rot = KDL::Vector(0.0, 0.0, 0.0);

     	   if (trajectory_ == "circular") {
     	       // Adjust control gains for circular trajectory
      	      Kpp = 500;
      	      Kpo = 400;
      	      Kdp = 100;
       	      Kdo = 100;
      	   } 

     	   // Compute torques using idCntr function
     	   Eigen::VectorXd torques = controller_->idCntr(desPos, desVel, desAcc, Kpp, Kpo, Kdp, Kdo);

      	  // Assign computed torques to joint effort commands
      	  for (int i = 0; i < torques.size(); ++i) {
      	      joint_efforts_cmd_(i) = torques(i);
      	      desired_commands_[i] = joint_efforts_cmd_(i);
      	      RCLCPP_INFO(this->get_logger(), "q_dot[%u]: %.6f", i, torques(i));
     	   }

    	    // Create and publish the message
    	    std_msgs::msg::Float64MultiArray cmd_msg;
   	     cmd_msg.data = desired_commands_;
    	    cmdPublisher_->publish(cmd_msg);

   	 } else {
    	    if (!maintenance_message_printed) {
    	        RCLCPP_INFO(this->get_logger(), "Maintenance mode...");
    	        maintenance_message_printed = true;
   	     }
    	    maintanance_mode();
   	 }
     }

     void maintanance_mode() {
   	 // Continue using idCntr to maintain the final position
  	 robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));

  	 // Get the current position of the manipulator
   	 KDL::Frame current_cart_pose = robot_->getEEFrame();

  	 // Desired position
  	 KDL::Frame desPos = current_cart_pose;

  	 // Desired velocity and acceleration set to zero
  	 KDL::Twist desVel;
  	 desVel.vel = KDL::Vector(0.0, 0.0, 0.0);
   	 desVel.rot = KDL::Vector(0.0, 0.0, 0.0);

  	 KDL::Twist desAcc;
   	 desAcc.vel = KDL::Vector(0.0, 0.0, 0.0);
   	 desAcc.rot = KDL::Vector(0.0, 0.0, 0.0);

    	// Control gains
    	double Kpp = 50;
    	double Kpo = 50;
    	double Kdp = 5;
    	double Kdo = 50;

    	// Compute torques using idCntr
    	Eigen::VectorXd torques = controller_->idCntr(desPos, desVel, desAcc, Kpp, Kpo, Kdp, Kdo);

    	// Assign computed torques to joint effort commands
    	for (int i = 0; i < torques.size(); ++i) {
        	joint_efforts_cmd_(i) = torques(i);
        	desired_commands_[i] = joint_efforts_cmd_(i);
    	}

    	// Create and publish the message
    	std_msgs::msg::Float64MultiArray cmd_msg;
    	cmd_msg.data = desired_commands_;
    	cmdPublisher_->publish(cmd_msg);
     }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointSubscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr aruco_pose_subscriber_;
    rclcpp::Publisher<FloatArray>::SharedPtr cmdPublisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<double> desired_commands_;
    KDL::JntArray joint_positions_;
    KDL::JntArray joint_velocities_;
    KDL::JntArray joint_positions_cmd_;
    KDL::JntArray joint_velocities_cmd_;
    KDL::JntArray joint_efforts_cmd_;

    std::shared_ptr<KDLRobot> robot_;
    KDLPlanner planner_;
    std::shared_ptr<KDLController> controller_;

    int iteration_;
    double t_;
    std::string cmd_interface_;

    // Additional variables
    KDL::Frame aruco_pose_;
    Eigen::Vector3d Po_w_;
    Eigen::VectorXd q_init_;
    Eigen::Vector3d error_ori;
    bool joint_state_available_;
    bool aruco_pose_available_;   
    bool posizione_iniziale_raggiunta_;    
    std::string task_;
    std::string trajectory_;
    KDL::Frame init_cart_pose_; 
    KDL::JntArray init_joint_positions_;
    double soglia_errore_; // Soglia per determinare se la posizione iniziale è stata raggiunta
    bool maintenance_message_printed = false; // Variabile di stato
    Eigen::Matrix3d R_ee_des;
    int trajRadius;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Iiwa_pub_sub>());
    rclcpp::shutdown();
    return 0;
}

