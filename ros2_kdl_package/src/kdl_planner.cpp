#include "kdl_planner.h"
#include <stdexcept>
#include <cmath>

KDLPlanner::KDLPlanner() {}

KDLPlanner::KDLPlanner(double trajDuration, double accDuration, const Eigen::Vector3d& trajInit, const Eigen::Vector3d& trajEnd)
    : trajDuration_(trajDuration), accDuration_(accDuration), trajInit_(trajInit), trajEnd_(trajEnd), trajectory_type_(LINEAR) {}

KDLPlanner::KDLPlanner(double trajDuration, const Eigen::Vector3d& trajInit, double trajRadius)
    : trajDuration_(trajDuration), accDuration_(trajDuration / 4.0), trajInit_(trajInit), trajRadius_(trajRadius), trajectory_type_(CIRCULAR) {}

void KDLPlanner::setProfileType(ProfileType profile) {
    profile_type_ = profile;
}

void KDLPlanner::setTrajectoryType(TrajectoryType type) {
    trajectory_type_ = type;
}

void KDLPlanner::trapezoidal_vel(double t, double tc, double tf, double& s, double& s_dot, double& s_ddot) {
    double acc = 1.0 / (tc * (tf - tc));
    if (t <= tc) {
        s = 0.5 * acc * t * t;
        s_dot = acc * t;
        s_ddot = acc;
    } else if (t <= tf - tc) {
        s = acc * tc * (t - tc / 2);
        s_dot = acc * tc;
        s_ddot = 0.0;
    } else if (t <= tf) {
        double t_dec = tf - t;
        s = 1.0 - 0.5 * acc * t_dec * t_dec;
        s_dot = acc * t_dec;
        s_ddot = -acc;
    } else {
        s = 1.0;
        s_dot = 0.0;
        s_ddot = 0.0;
    }
}

void KDLPlanner::cubic_polynomial(double t, double tf, double& s, double& s_dot, double& s_ddot) {
    // Boundary conditions
    double s0 = 0.0;  // Initial position
    double sf = 1.0;  // Final position
    double v0 = 0.0;  // Initial velocity
    double vf = 0.0;  // Final velocity

    // Compute cubic polynomial coefficients
    double a3 = (2 * s0 - 2 * sf + tf * vf + tf * v0) / (tf * tf * tf);
    double a2 = (3 * sf - 3 * s0 - 2 * tf * v0 - tf * vf) / (tf * tf);
    double a1 = v0;
    double a0 = s0;

    if (t <= tf) {
        s = a3 * t * t * t + a2 * t * t + a1 * t + a0;
        s_dot = 3 * a3 * t * t + 2 * a2 * t + a1;
        s_ddot = 6 * a3 * t + 2 * a2;
    } else {
        s = 1.0;
        s_dot = 0.0;
        s_ddot = 0.0;
    }
}

trajectory_point KDLPlanner::compute_trajectory(double time) {
    trajectory_point traj;
    double s, s_dot, s_ddot;

    // Choose trajectory with velocity profiles
    if (profile_type_ == CUBIC) {
        cubic_polynomial(time, trajDuration_, s, s_dot, s_ddot);
    } else if (profile_type_ == TRAPEZOIDAL) {
        trapezoidal_vel(time, accDuration_, trajDuration_, s, s_dot, s_ddot);
    }

    if (trajectory_type_ == CIRCULAR) {
        traj.pos << trajInit_.x(), trajInit_.y() - trajRadius_ * cos(2 * M_PI * s), trajInit_.z() - trajRadius_ * sin(2 * M_PI * s);
        traj.vel << 0, trajRadius_ * 2 * M_PI * s_dot * sin(2 * M_PI * s), -trajRadius_ * 2 * M_PI * s_dot * cos(2 * M_PI * s);
        traj.acc << 0, trajRadius_ * 2 * M_PI * (s_ddot * sin(2 * M_PI * s) + s_dot * s_dot * cos(2 * M_PI * s)), -trajRadius_ * 2 * M_PI * (s_ddot * cos(2 * M_PI * s) - s_dot * s_dot * sin(2 * M_PI * s));
    } else if (trajectory_type_ == LINEAR) {
        Eigen::Vector3d direction = trajEnd_ - trajInit_;
        traj.pos = trajInit_ + s * direction;
        traj.vel = s_dot * direction;
        traj.acc = s_ddot * direction;
    }

    return traj;
}
