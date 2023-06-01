#ifndef STARQ_LEG_HPP_
#define STARQ_LEG_HPP_

#include <vector>

namespace starq {

class Leg {

public:

    Leg() {}

    std::size_t size() { return motor_ids_.size(); }

    std::vector<int> motor_ids() { return motor_ids_; }

    std::vector<float> next_position() {
        // TODO: Inverse Kinematics
    }

private:

    std::vector<int> motor_ids_;
    // KinematicModel kinematics_;
    // Gait current_gait_;

};

}

#endif