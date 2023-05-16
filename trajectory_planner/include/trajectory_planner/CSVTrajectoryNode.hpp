#ifndef STARQ_CSV_TRAJECTORY_PLANNER_NODE_HPP_
#define STARQ_CSV_TRAJECTORY_PLANNER_NODE_HPP_

#include <trajectory_planner/types.hpp>
#include <string>

namespace starq {

class CSVTrajectoryPlannerNode {

public:

    CSVTrajectoryPlannerNode(const std::string& csv_folder)
    : csv_folder_(csv_folder) {}

    // Set the trajectory run frequency
    void set_frequency(const int frequency) {
        frequency_ = frequency;
    }

private:

    const std::string csv_folder_;

    int frequency_;
    

};

}

#endif