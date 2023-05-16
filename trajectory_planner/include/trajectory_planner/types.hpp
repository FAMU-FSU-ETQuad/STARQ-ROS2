#ifndef STARQ_TRAJECTORY_PLANNER_TYPES_HPP_
#define STARQ_TRAJECTORY_PLANNER_TYPES_HPP_

#include <vector>
#include <array>
#include <list>

namespace starq {

    // General objects
    using Trajectory = std::list<std::array<std::array<float,3>,4>>;

    // Cartesian objects
    using Point3D = std::array<float,3>;
    using LegPoints = std::array<Point3D,4>;
    using PointTrajectory = std::list<LegPoints>;

    using Velocity3D = std::array<float,3>;
    using LegVelocities = std::array<Velocity3D,4>;
    using VelocityTrajectory = std::list<LegVelocities>;

    // Angle objects
    using Angle3D = std::array<float,3>;
    using LegAngles = std::array<Angle3D,4>;
    using AngleTrajectory = std::list<LegAngles>;

    using Spin3D = std::array<float,3>;
    using LegSpins = std::array<Spin3D,4>;
    using SpinTrajectory = std::list<LegSpins>;

    // Message types
    using MotorCommand = std::vector<float>;

}

#endif