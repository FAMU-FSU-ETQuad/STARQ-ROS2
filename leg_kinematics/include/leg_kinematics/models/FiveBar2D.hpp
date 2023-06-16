#ifndef STARQ_KINEMATIC_MODEL_FIVEBAR2D_HPP_
#define STARQ_KINEMATIC_MODEL_FIVEBAR2D_HPP_

#include <leg_kinematics/types.hpp>
#include <leg_kinematics/KinematicModel.hpp>

namespace starq {

class FiveBar2DModel : public KinematicModel {

enum CartesianIndices {X, Y, Z};
enum MotorAngleIndices {A, B, C};

public:

    FiveBar2DModel(const float L1, const float L2)
    : L1_(L1), L2_(L2) {}

    MotorPosition get_inverse(const LegPosition& point) override {
        MotorPosition angles(2);
        const float theta0 = std::atan2(point[Y], point[X]);
        const float theta1 = std::atan2(point[Y], -point[X]);
        const float R = std::sqrt(point[X]*point[X] + point[Y]*point[Y]);
        const float alpha = std::acos((R*R + L1_*L1_ - L2_*L2_) / (2.0f*R*L1_));
        angles[A] = (theta0+alpha)/(2.0*M_PI);
        angles[B] = (theta1+alpha)/(2.0*M_PI);
        return angles;
    }

    LegPosition get_forward(const MotorPosition& angles) override {
        // TODO
        return {0.0f, 0.0f, 0.0f};
    }

private:

    const float L1_, L2_;

};

}

#endif