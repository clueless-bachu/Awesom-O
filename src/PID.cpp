/**
 * @file Controller.h
 * @author Sneha Nayak
 * @author Vishnuu
 * @author Vasista
 * @brief
 * @date 2020-12-01
 * @copyright Copyright (c) 2020
 * 
 */
#include "PID.h"

PID::PID(float kp, float ki, float kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    Integrator = 0;
    Derivator = 0;
    Integrator_max = 500;
    Integrator_min = -500;
    clearPID();
}

PID::~PID() {
    clearPID();
}

void PID::clearPID() {
    P_term_ = 0;
    I_term_ = 0;
    D_term_ = 0;
    prev_error_ = 0;
}

float PID::control(float input) {
    P_term_ = kp_ * input;
    D_term_ = kd_ * (input - Derivator);
    Derivator = input;

    Integrator = Integrator + input;

        if (Integrator > Integrator_max)
            Integrator = Integrator_max;
        else if (Integrator < Integrator_min)
            Integrator = Integrator_min;

        I_term_ = Integrator * ki_;

        float PID = P_term_ + I_term_ + D_term_;

        return PID;
}
