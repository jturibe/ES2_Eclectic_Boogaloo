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
    c_err = 0;
    past_rota_err = 0;
}

//// FUNCTION: Compute power for VELOCITY using PID
float PWMController::setVelocity(float error_term){
    // Calculate the proportional term
    y_ps = k_ps*error_term;

    // Calculate the integral term
    c_err += error_term;
    y_is = c_err*k_is;

    y_is = y_is>y_is_limit ? y_is_limit:y_is;
    // y_is = y_is<-y_is_limit ?-y_is_limit:y_is; //

    //Calculate PWM control
    y_s = y_ps + y_is; //+ y_ds

    // Convert power term to valid PWM
    lead = y_s < 0 ? -2:2;
    y_s = abs(y_s) > PWM_PRD ? PWM_PRD : y_s;

    return abs(y_s);
}

//// FUNCTION: Compute power for ROTATION using PID
float PWMController::setRotation(float error_term){
    // Calculate the proportional term
    y_pr = k_pr*error_term;

    // Calculate the differential term
    y_dr = k_dr*(error_term - past_rota_err);

    // Calculate PWM control
    y_r = y_pr + y_dr;

    // Convert power term to valid PWM
    lead = y_r < 0 ? -2:2;
    y_r = abs(y_r) > PWM_PRD ? PWM_PRD : y_r;

    // Update value of previous error
    past_rota_err = error_term;

    return abs(y_r);
}

float PWMController::pwmController(){
    float power;
    maxVelocity_mutex.lock();
    float velocity_error = maxVelocity-velocity;
    maxVelocity_mutex.unlock();
    selectRotations_mutex.lock();
    float rotation_error = selectRotations-((float)motorPosition)/6;
    selectRotations_mutex.unlock();
    float y_s = this->setVelocity(velocity_error);
    float y_r = this->setRotation(rotation_error);
    if(velocity < 0){
        power = max(y_s,y_r);
    } else {
        power = min(y_s,y_r);
    }
    return power;
}
