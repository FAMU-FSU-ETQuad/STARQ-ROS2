#ifndef STARQ_IK_TYPES_HPP_
#define STARQ_IK_TYPES_HPP_

#include <string>
#include <memory>
#include <vector>

namespace starq {

using LegPosition = std::vector<float>;
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