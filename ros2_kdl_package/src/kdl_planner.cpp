#include "kdl_planner.h"


KDLPlanner::KDLPlanner(){}

KDLPlanner::KDLPlanner(double _maxVel, double _maxAcc)
{
    velpref_ = new KDL::VelocityProfile_Trap(_maxVel,_maxAcc);
}

KDLPlanner::KDLPlanner(double _trajDuration, double _accDuration, 
                       Eigen::Vector3d _trajInit, Eigen::Vector3d _trajEnd, 
                       Eigen::Quaterniond& orientationInit, Eigen::Quaterniond& orientationEnd)
{
    trajDuration_ = _trajDuration;
    accDuration_ = _accDuration;
    trajInit_ = _trajInit;
    trajEnd_ = _trajEnd;
    orientationInit_ = orientationInit; 
    orientationEnd_ = orientationEnd;
}


void KDLPlanner::CreateTrajectoryFromFrames(std::vector<KDL::Frame> &_frames,
                                            double _radius, double _eqRadius
                                            )
{
    path_ = new KDL::Path_RoundedComposite(_radius,_eqRadius,new KDL::RotationalInterpolation_SingleAxis());

    for (unsigned int i = 0; i < _frames.size(); i++)
    {
        path_->Add(_frames[i]);
    }
    path_->Finish();

    velpref_->SetProfile(0,path_->PathLength());
    traject_ = new KDL::Trajectory_Segment(path_, velpref_);
}

void KDLPlanner::createCircPath(KDL::Frame &_F_start,
                                KDL::Vector &_V_centre,
                                KDL::Vector& _V_base_p,
                                KDL::Rotation& _R_base_end,
                                double alpha,
                                double eqradius
                                )
{
    KDL::RotationalInterpolation_SingleAxis* otraj;
    otraj = new KDL::RotationalInterpolation_SingleAxis();
    otraj->SetStartEnd(_F_start.M,_R_base_end);
    path_circle_ = new KDL::Path_Circle(_F_start,
                                        _V_centre,
                                        _V_base_p,
                                        _R_base_end,
                                        alpha,
                                        otraj,
                                        eqradius);
    velpref_->SetProfile(0,path_circle_->PathLength());
    traject_ = new KDL::Trajectory_Segment(path_circle_, velpref_);
}

KDL::Trajectory* KDLPlanner::getTrajectory()
{
	return traject_;
}

// Nuovo costruttore per traiettoria circolare
KDLPlanner::KDLPlanner(double _trajDuration, double _accDuration, 
                       Eigen::Vector3d _trajInit, double _trajRadius)
{
    trajDuration_ = _trajDuration;
    accDuration_ = _accDuration;
    trajInit_ = _trajInit;
    trajRadius_ = _trajRadius;
    circular_trajectory_ = true;
}

trajectory_point KDLPlanner::compute_trajectory(double time)
{
    trajectory_point traj;

    // Profilo trapezoidale per s, s_dot, s_ddot
    double s, s_dot, s_ddot;

    if (time <= accDuration_)
    {
        // Fase di accelerazione
        s = 0.5 * std::pow(time / accDuration_, 2);
        s_dot = time / std::pow(accDuration_, 2);
        s_ddot = 1.0 / std::pow(accDuration_, 2);
    }
    else if (time <= trajDuration_ - accDuration_)
    {
        // Fase di velocità costante
        s = (time - accDuration_ / 2.0) / trajDuration_;
        s_dot = 1.0 / trajDuration_;
        s_ddot = 0.0;
    }
    else if (time <= trajDuration_)
    {
        // Fase di decelerazione
        double t_rem = trajDuration_ - time;
        s = 1.0 - 0.5 * std::pow(t_rem / accDuration_, 2);
        s_dot = t_rem / std::pow(accDuration_, 2);
        s_ddot = -1.0 / std::pow(accDuration_, 2);
    }
    else
    {
        // Traiettoria completata
        s = 1.0;
        s_dot = 0.0;
        s_ddot = 0.0;
    }

    if (!circular_trajectory_)
    {
        // --- TRAIETTORIA LINEARE (CODICE ESISTENTE) ---
        Eigen::Vector3d ddot_traj_c = -1.0 / (std::pow(accDuration_, 2) - trajDuration_ * accDuration_) * (trajEnd_ - trajInit_);

        if (time <= accDuration_)
        {
            traj.pos = trajInit_ + 0.5 * ddot_traj_c * std::pow(time, 2);
            traj.vel = ddot_traj_c * time;
            traj.acc = ddot_traj_c;
        }
        else if (time <= trajDuration_ - accDuration_)
        {
            traj.pos = trajInit_ + ddot_traj_c * accDuration_ * (time - accDuration_ / 2);
            traj.vel = ddot_traj_c * accDuration_;
            traj.acc = Eigen::Vector3d::Zero();
        }
        else
        {
            traj.pos = trajEnd_ - 0.5 * ddot_traj_c * std::pow(trajDuration_ - time, 2);
            traj.vel = ddot_traj_c * (trajDuration_ - time);
            traj.acc = -ddot_traj_c;
        }

        // Interpolazione quaternioni
        traj.orientation = orientationInit_.slerp(s, orientationEnd_);
    }
    else
    {
        // --- TRAIETTORIA CIRCOLARE ---
        const double two_pi = 2 * M_PI;

        // Calcola posizione
        traj.pos << trajInit_.x(),
                    trajInit_.y() - trajRadius_ * cos(two_pi * s),
                    trajInit_.z() - trajRadius_ * sin(two_pi * s);

        // Calcola velocità
        traj.vel << 0,
                    trajRadius_ * two_pi * s_dot * sin(two_pi * s),
                    -trajRadius_ * two_pi * s_dot * cos(two_pi * s);

        // Calcola accelerazione
        traj.acc << 0,
                    trajRadius_ * two_pi * (s_ddot * sin(two_pi * s) + s_dot * s_dot * cos(two_pi * s)),
                    -trajRadius_ * two_pi * (s_ddot * cos(two_pi * s) - s_dot * s_dot * sin(two_pi * s));

    }

    return traj;
}

