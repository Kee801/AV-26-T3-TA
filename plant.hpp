#pragma once
// Your model of the actuator, reconstructed from the decoded CSVs. This is
// the Part B deliverable, alongside your written notes.
//
// Implement step(): given a commanded velocity and a timestep, return the
// measured output angle. The placeholder below is a bare integrator with
// gain 1 -- NOT the real actuator. Replace it with what the data shows
// (dynamics, gain, any nonlinearity, any lag), or the harness proves nothing.

#include <cmath>

struct Plant {
    // add whatever state your model needs (velocity, motor-side angle, ...)
    double angle = 0.0;
    double plant_gain = 1.30;
    double time_constant = 0.067;
    double backlash_width =  5;

    double rate = 0;
    double measured = 0;

    // u_cmd : commanded velocity, deg/s
    // dt    : timestep, seconds
    // return: measured output angle, deg
    double step(double u_cmd, double dt) {

        rate += (plant_gain * u_cmd - rate) * (dt/time_constant);

        angle += rate * dt;                   // placeholder dynamics -- replace this
        if (angle - measured > backlash_width / 2.0)
        {
            measured = angle - backlash_width / 2.0;
        } 
        else if (angle - measured < -backlash_width / 2.0) 
        {
            measured = angle + backlash_width / 2.0;
        }
        return std::round(measured / 0.1) * 0.1;  // the sensor reads to 0.1 deg
    }

    void reset() { angle = 0.0; rate = 0.0; measured = 0.0; }
};
