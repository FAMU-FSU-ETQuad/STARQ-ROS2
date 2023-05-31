#ifndef STARQ_KINEMATIC_MODEL_HPP_
#define STARQ_KINEMATIC_MODEL_HPP_

#include <vector>
#include <string>

namespace starq {

class KinematicModel {

using LegPosition = std::vector<float>;
using MotorPosition = std::vector<float>;

public:

    KinematicModel() {}

    virtual std::string get_name() = 0;

    virtual LegPosition get_forward(const MotorPosition& point) = 0;

    virtual MotorPosition get_inverse(const LegPosition& point) = 0;

};

}

#endif