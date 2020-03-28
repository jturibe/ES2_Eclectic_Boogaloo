#include "motor_control.h"

////////////////////////////////////////////////////////////////////////////////
// GLOBAL VARIABLES and FUNCTIONS used specifically for controlling the motor.//
// See header file for FUNCTION DECLARATIONS and PIN DEFINITIONS.             //
// FUNCTIONS in file:                                                         //
// - motorOut(int8_t driveState)                                              //
// - readRotorState()                                                         //
// - motorHome()                                                              //
// - motorControlISR()                                                        //
// - motorCtrlTick()                                                          //
// - motorCtrlFn()                                                            //
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//////////////////////////// GLOBAL VARIABLES //////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

// Drive state to output table
const int8_t driveTable[] = {0x12,0x18,0x09,0x21,0x24,0x06,0x00,0x00};

// Mapping from interrupter inputs to sequential rotor states. 0x00 and 0x07 are not valid
const int8_t stateMap[] = {0x07,0x05,0x03,0x04,0x01,0x00,0x02,0x07};
//const int8_t stateMap[] = {0x07,0x01,0x03,0x02,0x05,0x00,0x04,0x07}; //Alternative if phase order of input or drive is reversed

// Phase lead to make motor spin
volatile int8_t lead = 2;  //2 for forwards, -2 for backwards
int32_t PWM_PRD = 2500;

// Original and Starting state of the motor, used to implement motor rotation
int8_t orState = 0;

// Photointerrupter inputs
InterruptIn I1(I1pin);
InterruptIn I2(I2pin);
InterruptIn I3(I3pin);

// Motor Drive outputs
DigitalOut L1L(L1Lpin);
DigitalOut L1H(L1Hpin);
DigitalOut L2L(L2Lpin);
DigitalOut L2H(L2Hpin);
DigitalOut L3L(L3Lpin);
DigitalOut L3H(L3Hpin);

// TP OUT ????????????
DigitalOut TP1(TP1pin);

// Pulse width modulator for motor output
PwmOut MotorPWM(PWMpin);

//Status LED
DigitalOut led1(LED1);

// PWM torque - currently unused
volatile uint64_t pwmTorque;

// Motor position in amount of states, to calculate rotations must divide by 6
volatile int64_t motorPosition = 0;

volatile float velocity;

// PWM Controller to set required pulsewidth, and hence motor power.
PWMController pwmcontrol;

////////////////////////////////////////////////////////////////////////////////
//////////////////////////// FUNCTIONS /////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

//// FUNCTION: Set photointerrupter pins for calling motorControlISR
void setISRPhotoInterruptors(){
    MotorPWM.period_us(PWM_PRD);
    MotorPWM.pulsewidth_us(PWM_PRD);
    orState = motorHome();
    I1.rise(&motorControlISR);
    I1.fall(&motorControlISR);
    I2.rise(&motorControlISR);
    I2.fall(&motorControlISR);
    I3.rise(&motorControlISR);
    I3.fall(&motorControlISR);
}

//// FUNCTION: Set a given drive state
void motorOut(int8_t driveState){

    //Lookup the output byte from the drive state.
    int8_t driveOut = driveTable[driveState & 0x07];

    //Turn off first
    if (~driveOut & 0x01) L1L = 0;
    if (~driveOut & 0x02) L1H = 1;
    if (~driveOut & 0x04) L2L = 0;
    if (~driveOut & 0x08) L2H = 1;
    if (~driveOut & 0x10) L3L = 0;
    if (~driveOut & 0x20) L3H = 1;

    //Then turn on
    if (driveOut & 0x01) L1L = 1;
    if (driveOut & 0x02) L1H = 0;
    if (driveOut & 0x04) L2L = 1;
    if (driveOut & 0x08) L2H = 0;
    if (driveOut & 0x10) L3L = 1;
    if (driveOut & 0x20) L3H = 0;
}

//// FUNCTION: Convert photointerrupter inputs to a rotor state
inline int8_t readRotorState(){
    return stateMap[I1 + 2*I2 + 4*I3];
}


//// FUNCTION: Basic synchronisation routine
int8_t motorHome() {
    //Put the motor in drive state 0 and wait for it to stabilise
    motorOut(0);
    wait(2.0);

    //Get the rotor state
    return readRotorState();
}

//// FUNCTION: Interrupt Routine. Called on every change in encoder input.
//// Essentially runs the motor.
void motorControlISR(){
    static int8_t intStateOld;
    int8_t intState = readRotorState();

    // setVelocity(y_s); ??????

    motorOut((intState-orState+lead+6)%6); //+6 to make sure the remainder is positive
    if (intState == 4 && intStateOld == 3) TP1 = !TP1;
    if(intState - intStateOld == 5){
        motorPosition--;
    } else if (intState - intStateOld == -5){
        motorPosition++;
    } else{
        motorPosition += intState - intStateOld;
    }
    intStateOld = intState;
}

//// FUNCTION: Ticker function. Enables while loop in motorCtrlFn every 100ms.
//// Part of motor control thread: motorCtrlT
void motorCtrlTick(){
    // Send signal 0x1 to motor control thread
    motorCtrlT.signal_set(0x1);
}

//// FUNCTION: Computes velocity and rotational position.
//// Uses PWMController class to set motor power based on computed values.
//// Part of motor control thread: motorCtrlT
void motorCtrlFn(){
    // Initialise required variables
    int64_t old_position = motorPosition;
    Ticker motorCtrlTicker;
    Timer timer;
    float mult;
    float position_change;
    float velocity;
    float rotation_error;
    float error_term;
    int iter = 0;
    // Set ticker to call motorCtrlTick every 100 ms
    motorCtrlTicker.attach_us(&motorCtrlTick,100000);
    timer.start();
    while(1){
        // Stall thread till signal 0x1 is received. Runs while loop every 100ms
        motorCtrlT.signal_wait(0x1);
        // Calculate actual current velocity
        position_change = motorPosition - old_position;
        velocity = position_change/timer.read()/6;
        // Reset timer to measure correct period between motor position readings
        timer.reset();
        // Calculate rotation error and set power
        // rotation_error = selectRotations - ((float)motorPosition)/6;
        // pwmcontrol.setRotation(rotation_error);
        // Calculate velocity error and set power
        error_term = (maxVelocity - velocity);
        //MotorPWM.pulsewidth_us(pwmcontrol.setVelocity(error_term));
        MotorPWM.pulsewidth_us(pwmcontrol.setRotation(selectRotations-((float)motorPosition)/6));
        if(iter == 9){
             char message[150];
             // // sprintf(message,"MaxVelocity: %f, Motor Velocity: %f, Motor Power: %f\n\r",maxVelocity,velocity,pwmcontrol.y_s);
             sprintf(message, "Motor Velocity: %f, Motor Position: %f, Selected Position: %f, Set power: %f\n\r",velocity,((float)motorPosition)/6,selectRotations,pwmcontrol.y_r);
             putMessage(message);
            // char message2[150];
            // sprintf(message2, "Position error: %f, Proportional term: %f, Full term: %f, Lead: %d\n\r",rotation_error, pwmcontrol.y_pr, pwmcontrol.y_r, lead);
            // putMessage(message2);
            // char message3[50];
            // sprintf(message3, "PWM: %f\n\r",MotorPWM.read());
            // putMessage(message3);

            iter = 0;
        }
        iter++;
        old_position = motorPosition;
    }
}
