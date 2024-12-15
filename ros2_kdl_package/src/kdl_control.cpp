#include "kdl_control.h"
#include "utils.h"

KDLController::KDLController(KDLRobot &_robot)
{
    robot_ = &_robot;
}

Eigen::VectorXd KDLController::idCntr_2(KDL::JntArray &_qd,
                                      KDL::JntArray &_dqd,
                                      KDL::JntArray &_ddqd,
                                      double _Kp, double _Kd)
{
    // read current state
    Eigen::VectorXd q = robot_->getJntValues();
    Eigen::VectorXd dq = robot_->getJntVelocities();

    // calculate errors
    Eigen::VectorXd e = _qd.data - q;
    Eigen::VectorXd de = _dqd.data - dq;

    Eigen::VectorXd ddqd = _ddqd.data;
    return robot_->getJsim() * (ddqd + _Kd*de + _Kp*e)
            + robot_->getCoriolis() + robot_->getGravity() /*friction compensation?*/;
}

Eigen::VectorXd KDLController::idCntr(KDL::Frame& _desPos, KDL::Twist& _desVel, KDL::Twist& _desAcc,
                                      double _Kpp, double _Kpo, double _Kdp, double _Kdo) {
    // Retrieve the current state of the manipulator
    KDL::Frame ee_frame = robot_->getEEFrame();                // Current position of the end-effector
    KDL::Twist ee_velocity = robot_->getEEVelocity();          // Current velocity of the end-effector
    Eigen::MatrixXd jacobian = robot_->getEEJacobian().data;   // Current analytical Jacobian
    Eigen::VectorXd jacobian_dot_q_dot = robot_->getEEJacDotqDot();  // Jacobian derivative * q_dot --> check robot.cpp
    
    // Compute errors in the operational space
    Vector6d e;    
    Vector6d edot; 
    computeErrors(_desPos, ee_frame, _desVel, ee_velocity, e, edot);

    // Build the gain matrices
    Matrix6d Kp, Kd;
    Kp.setZero();
    Kd.setZero();

    for (int i = 0; i < 3; ++i) {
        Kp(i, i) = _Kpp; 
        Kd(i, i) = _Kdp; 
    }
    for (int i = 3; i < 6; ++i) {
        Kp(i, i) = _Kpo; 
        Kd(i, i) = _Kdo; 
    }

    // Control output combining desired acceleration and PD terms
    Eigen::VectorXd desAcc_ = toEigen(_desAcc);
    Eigen::VectorXd control_output = desAcc_ + Kd * edot + Kp * e;
    //std::cout << "[DEBUG] control_output: " << control_output.transpose() << std::endl;

    // Desired joint accelerations
    Eigen::VectorXd y = pseudoinverse(jacobian) * (control_output - jacobian_dot_q_dot);

    // Compute control law
    Eigen::MatrixXd Jsim = robot_->getJsim();                // Inertia matrix, B
    
    Eigen::VectorXd n = robot_->getCoriolis() + robot_->getGravity();               // Coriolis + Gravity terms

    Eigen::VectorXd tau = Jsim * y + n;                          // Desired torques

    // Debug final torques
    //std::cout << "[DEBUG] Computed torques (tau): " << tau.transpose() << std::endl;

    return tau;
}



