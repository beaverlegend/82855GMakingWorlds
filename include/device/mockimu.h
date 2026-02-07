#pragma once

#include "main.h"

class MockIMU : public pros::Imu
{
public:
    MockIMU(int port, double gain)
        : pros::Imu(port), imu_gain(gain) {}

    double get_rotation() const override
    {
        double raw = pros::Imu::get_rotation();
        if (raw == PROS_ERR_F)
            return NAN;
        return raw * imu_gain;
    }

private:
    double imu_gain;
};