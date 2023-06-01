#ifndef STARQ_GAIT_HPP_
#define STARQ_GAIT_HPP_

#include <trajectory_planner/types.hpp>
#include <string>
#include <map>

namespace starq {

struct Gait {
    Trajectory trajectory;
    double frequency;
};

class GaitLoader {

public:

    GaitLoader() {}

    Gait loadGaitFromCSV(const std::string& load_file) {

    }

private:

    std::map<std::string, Gait> loaded_gaits_cache_;

};

}

#endif