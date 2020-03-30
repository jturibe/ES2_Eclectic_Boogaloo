#include "pwm_controller.h"

////////////////////////////////////////////////////////////////////////////////
// PWM CONTROLLER CLASS function implementations:                             //
// PWMController()                                                            //
// setVelocity(float error_term)                                              //
// setRotation(float error_term)                                              //
////////////////////////////////////////////////////////////////////////////////

//// FUNCTION: Initialise variables to required starting values
PWMController::PWMController(){
    // Initialise VELOCITY PID terms
    y_s = 0;
    y_ps = 0;
    y_is = 0;
    y_ds = 0;

    // Initialise ROTATION PID terms
    y_r = 0;
    y_pr = 0;
    y_dr = 0;

    // Initialise ERROR TERMS
    s_err = 0;
    r_err = 0;
    past_rota_err = 0;
}


//// FUNCTION: Compute power for VELOCITY using PID
float PWMController::setVelocity(float error_term){
    // Calculate the proportional term
    y_ps = k_ps*error_term;

    // Calculate the integral term
    s_err += error_term;

    int sign = (s_err < 0)? -1: 1;

    //limit the error to a maximum
    s_err = abs(s_err)>y_is_limit/k_is ? sign*y_is_limit/k_is:s_err;

    y_is = s_err*k_is;

    // y_is = y_is<-y_is_limit ?-y_is_limit:y_is; //

    //Calculate PWM control
    y_s = y_ps + y_is; //+ y_ds

    return y_s;
}

//// FUNCTION: Compute power for ROTATION using PID
float PWMController::setRotation(float error_term, float time){
    // Calculate the proportional term
    y_pr = k_pr*error_term;

    //Calculate the integral term
    // r_err = abs(error_term)<0.5 ? 0 :r_err + error_term; // cancel error if position is within range
    // r_err = abs(r_err)>1400/k_ir ? sgn(r_err)*1400/k_ir:r_err;
    // y_ir = r_err*k_ir; //limit y_ir

    // Calculate the differential term
    y_dr = k_dr*(error_term - past_rota_err)/time;

    // Calculate PWM control
    y_r = y_pr + y_dr;// + y_ir;

    // Update value of previous error
    past_rota_err = error_term;

    return y_r;
}

float PWMController::pwmController(){
    float power;
    float velocity_error = maxVelocity-velocity;
    float rotation_error = selectRotations-((float)motorPosition)/6;
    float y_s_loc = setVelocity(velocity_error);
    float y_r_loc = setRotation(rotation_error, 1);
    if(velocity < 0){
        power = max(y_s_loc,y_r_loc);
    } else {
        power = min(y_s_loc,y_r_loc);
    }
    return power;
}
