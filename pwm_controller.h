#ifndef PWM_CONTROLLER_H
#define PWM_CONTROLLER_H

#include <mutex>
#include "mbed.h"
#include "messaging.h"
#include "motor_control.h"

////////////////////////////////////////////////////////////////////////////////
// PWM CONTROLLER CLASS declaration in charge of performing PID CONTROL to    //
// reach desired VELOCITY or desired amount of ROTATION                       //
////////////////////////////////////////////////////////////////////////////////

///// PWM CONTROLLER CLASS
class PWMController{

public:
    /// VELOCITY PID TERMS
    float y_s; // Final PWM VELOCITY control value = y_ps + y_is + y_ds
    float y_ps; // Proportional VELOCITY term
    float y_is; // Integral VELOCITY term
    float y_ds; // Differential VELOCITY term

    /// ROTATION PID TERMS
    float y_r; // Final PWM  ROTATION control value = y_p + y_dr
    float y_pr; // Proportional ROTATION term
    float y_dr; // Differential ROTATION term
    float y_ir; // Differential ROTATION term

    /// VELOCITY CONSTANTS
    float k_ps = 33; // Proportional VELOCITY constant
    float k_is = 0.17; // Integral VELOCITY constant
    float k_ds = 0; // Differential VELOCITY constant

    /// ROTATION CONSTANTS
    float k_pr = 15;//Proportional ROTATION constant
    float k_ir = 0; // Integral ROTATION constant
    float k_dr = 17; // Differential ROTATION constant

    /// ERROR TERMS
    float s_err; // Cumulative ERROR for speed
    float r_err; // Cumulative ERROR for rotation
    float past_rota_err; // Previous ROTATION ERROR

    /// INTEGRAL LIMIT
    float y_is_limit = 1500; // Integral limit to prevent overshoot

    //// FUNCTION DECLARATIONS
    PWMController(); // Class constructor: Initialises terms
    float setVelocity(float error_term); // Compute power for VELOCITY using PID
    float setRotation(float error_term, float time); // Compute power for ROTATION using PID
    float pwmController();

};

#endif
