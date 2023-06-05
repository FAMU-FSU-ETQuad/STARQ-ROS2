#ifndef STARQ_KINEMATIC_MODEL_HPP_
#define STARQ_KINEMATIC_MODEL_HPP_

#include <leg_kinematics/types.hpp>
#include <math.h>

namespace starq {

class KinematicModel {

public:

    using Ptr = std::shared_ptr<KinematicModel>;

    KinematicModel() {}

    virtual MotorPosition get_inverse(const LegPosition& point) = 0;

    virtual LegPosition get_forward(const MotorPosition& angles) = 0;

};

}

#endif