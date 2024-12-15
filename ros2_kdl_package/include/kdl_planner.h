#ifndef KDLPlanner_H
#define KDLPlanner_H

#include <kdl/frames_io.hpp>
#include <kdl/frames.hpp>
#include <kdl/trajectory.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/trajectory_stationary.hpp>
#include <kdl/trajectory_composite.hpp>
#include <kdl/velocityprofile_trap.hpp>
#include <kdl/path_circle.hpp>
#include <kdl/path_roundedcomposite.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/utilities/error.h>
#include "Eigen/Dense"

struct trajectory_point {
    Eigen::Vector3d pos;
    Eigen::Vector3d vel;
    Eigen::Vector3d acc;
};

enum ProfileType {
    CUBIC,
    TRAPEZOIDAL
};

enum TrajectoryType {
    LINEAR,
    CIRCULAR
};

class KDLPlanner {
public:
    KDLPlanner();
    // Constructors for circular and linear trajectories
    KDLPlanner(double trajDuration, const Eigen::Vector3d& trajInit, double trajRadius);
    KDLPlanner(double trajDuration, double accDuration, const Eigen::Vector3d& trajInit, const Eigen::Vector3d& trajEnd);
    
    // Trapezoidal velocity function
    void trapezoidal_vel(double t, double tc, double tf, double& s, double& s_dot, double& s_ddot);
    // Cubic polynomial function
    void cubic_polynomial(double t, double tf, double& s, double& s_dot, double& s_ddot);
    
    // Set profile type and trajectory type
    void setProfileType(ProfileType profile);
    void setTrajectoryType(TrajectoryType type);
    
    trajectory_point compute_trajectory(double time);
    KDL::Trajectory* getTrajectory();

private:
    ProfileType profile_type_;
    TrajectoryType trajectory_type_;
    double trajDuration_;
    double accDuration_;
    Eigen::Vector3d trajInit_;
    Eigen::Vector3d trajEnd_;
    double trajRadius_;  // Radius of the circular trajectory
    
    KDL::Path_RoundedComposite* path_;
    KDL::Path_Circle* path_circle_;
    KDL::VelocityProfile* velpref_;
    KDL::Trajectory* traject_;
    trajectory_point p;
};

#endif

