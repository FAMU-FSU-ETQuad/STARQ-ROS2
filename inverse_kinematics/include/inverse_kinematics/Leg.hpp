#ifndef STARQ_LEG_HPP_
#define STARQ_LEG_HPP_

#include <inverse_kinematics/types.hpp>
#include <inverse_kinematics/KinematicModel.hpp>

namespace starq {

class Leg {

public:

    Leg(const int id, const std::string& name, const std::vector<int> motor_ids) 
    : id_(id), name_(name), motor_ids_(motor_ids) {}

    int id() { return id_; }

    std::string name() { return name_; }

    std::vector<int> motors() { return motor_ids_; }

    std::shared_ptr<KinematicModel> model() { return model_; }

private:

    int id_;
    std::string name_;
    std::vector<int> motor_ids_;
    std::shared_ptr<KinematicModel> model_;

};

}

#endif