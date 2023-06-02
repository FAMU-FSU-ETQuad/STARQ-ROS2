#ifndef STARQ_KINEMATIC_MODEL_HPP_
#define STARQ_KINEMATIC_MODEL_HPP_

#include <inverse_kinematics/types.hpp>

namespace starq {

class KinematicModel {

public:

    KinematicModel() {}

    virtual std::string get_name() = 0;

    virtual LegPosition get_forward(const MotorPosition& point) = 0;

    virtual MotorPosition get_inverse(const LegPosition& point) = 0;

};

}

#endif