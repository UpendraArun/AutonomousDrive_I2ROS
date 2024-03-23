#ifndef AUTONOMOUSDRIVING_SRC_CONTROLLER_PKG_SRC_TRAJECTORY_FOLLOWER_H
#define AUTONOMOUSDRIVING_SRC_CONTROLLER_PKG_SRC_TRAJECTORY_FOLLOWER_H

#include "pid.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

namespace Control
{
    class trajectory_follower
    {
    public:
        trajectory_follower();
        ~trajectory_follower();

        double CalcAcceleration();
        double CalcTurningAngle();

    private:
        double max_aceleration{8.0};
        double min_aceleration{8.0};
        double max_turning_angle{2.0};
        double min_turning_angle{2.0};
    };

    trajectory_follower::trajectory_follower(){};
    trajectory_follower::~trajectory_follower(){};
}

#endif
