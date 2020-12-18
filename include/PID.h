/**
 * @file PID.h
 * @author Sneha Nayak
 * @author Vishnuu
 * @author Vasista
 * @brief
 * @date 2020-12-01
 * @copyright Copyright (c) 2020
 * 
 */

#include <ros/ros.h>

class PID {
private:
    float kp_;            // P gain
    float ki_;            // I gain
    float kd_;            // D gain
    float prev_error_;      // Previous error
    double P_term_;          // Proportional term
    double I_term_;          // Integral term
    double D_term_;          // Differential term
    int Integrator, Derivator, Integrator_max, Integrator_min;

public:
    
    /**
     * @brief Constructor for class PID
     *      Initializes the controller gains and sets the time values
     * @param float kp : P gain
     * @param float ki : I gain
     * @param float kd : D gain
     * @return void
     *
    */
    PID(float kp, float ki, float kd);

    /**
     * @brief Destructor for class PID
     * @param void
     * @return void
     **/
    ~PID();

    /**
     * @brief clearController method to clear up the PID terms
     *          and the time and error
     * @param void
     * @return void
     */
    void clearPID();

    /**
     * @brief update method to update the value of the input
     *      i.e. generate output from the PID controller
     * @param float input : input to the controller
     * @return float : output of the controller
     */
    float control(float input);


};