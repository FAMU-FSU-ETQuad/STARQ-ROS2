#ifndef STARQ_IK_TYPES_HPP_
#define STARQ_IK_TYPES_HPP_

#include <string>
#include <memory>
#include <array>
#include <vector>

namespace starq {

using LegPosition = std::array<float, 3>;
using MotorPosition = std::vector<float>;

class KinematicModel;

struct Leg {
    int id;
    std::string name;
    std::vector<int> motor_ids;
    std::shared_ptr<KinematicModel> model;
};

}

#endif