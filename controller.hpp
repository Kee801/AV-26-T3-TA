#pragma once
// Implement Controller so that, given only the target angle, the last
// measured angle, and the timestep, it drives the system to the target --
// despite whatever nonlinearity you identified from the CSVs.
//
// This is the file you submit. You can add private members, helper methods,
// filters, whatever your design needs. We will never run your internals.


#include "controller_interface.hpp"

class Controller : public IController {
private:
    double integral = 0.0; // so that it doesnt reset to 0 everytime
    double previous_error = 0.0;
    double integral_limit = 2.0;
public:
    double update(double target, double measured, double dt) override {
        double error = target - measured;
        integral += error * dt;
        if (integral > integral_limit)
        {
            integral = integral_limit;
        }
        if (integral < -integral_limit)
        {
            integral = -integral_limit;
        }
        double derivative = (error - previous_error)/dt;

        double kp = 1.5; // TODO: replace with your design
        double ki = 0.01;
        double kd = 0.01;

        double P = kp * error;
        double I = ki * integral;
        double D = kd * derivative;

        double output = P+I+D;
        previous_error = error;

        return output;
    }

    void reset() override {
        // TODO: reset any internal state here, if you have any.
        integral = 0.0;
        previous_error = 0.0;
    }
};
