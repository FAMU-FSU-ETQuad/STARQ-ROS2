#ifndef STARQ_LEG_KINEMATICS_HPP_
#define STARQ_LEG_KINEMATICS_HPP_

#include <trajectory_planner/types.hpp>

namespace starq {

/*
    FORWARD KINEMATICS
*/

Point3D forward_kinematics(const Angle3D& angle) {
    // TODO
    return {0,0,0};
}

LegPoints forward_kinematics(const LegAngles& leg_angles) {
    LegPoints leg_points;
    for (int idx = 0; idx < leg_angles.size(); idx++)
        leg_points[idx] = forward_kinematics(leg_angles[idx]);
    return leg_points;
}

PointTrajectory forward_kinematics(const AngleTrajectory& traj_in) {
    PointTrajectory traj_out;
    for (auto leg_angle = traj_in.cbegin(); leg_angle != traj_in.cend(); ++leg_angle)
        traj_out.push_back(forward_kinematics(*leg_angle));
    return traj_out;
}

/*
    INVERSE KINEMATICS
*/

Angle3D inverse_kinematics(const Point3D& point) {
    // TODO
    return {0,0,0};
}

LegAngles inverse_kinematics(const LegPoints& leg_points) {
    LegAngles leg_angles;
    for (int idx = 0; idx < leg_points.size(); idx++)
        leg_angles[idx] = inverse_kinematics(leg_points[idx]);
    return leg_angles;
}

AngleTrajectory inverse_kinematics(const PointTrajectory& traj_in) {
    AngleTrajectory traj_out;
    for (auto leg_point = traj_in.cbegin(); leg_point != traj_in.cend(); ++leg_point)
        traj_out.push_back(inverse_kinematics(*leg_point));
    return traj_out;
}

}

#endif