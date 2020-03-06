#include "mbed.h"
#include "Timer.h"
#include "pwm_controller.h"

////////////////////////////////////////////////////////////////////////////////
// Header file containing all declarations of pins, global variables and      //
// functions that dictate motor control.                                      //
//                                                                            //
// Pin Definitions                                                            //
// Functions                                                                  //
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//////////////////////////////// PIN DEFINITIONS ///////////////////////////////
////////////////////////////////////////////////////////////////////////////////

// Photointerrupter input pins
#define I1pin D3
#define I2pin D6
#define I3pin D5

// Incremental encoder input pins
#define CHApin   D12
#define CHBpin   D11

// Motor Drive output pins   //Mask in output byte
#define L1Lpin D1           //0x01
#define L1Hpin A3           //0x02
#define L2Lpin D0           //0x04
#define L2Hpin A6          //0x08
#define L3Lpin D10           //0x10
#define L3Hpin D2          //0x20

// PWM output pin
#define PWMpin D9

// Motor current sense
#define MCSPpin   A1
#define MCSNpin   A0

// Test outputs
#define TP0pin D4
#define TP1pin D13
#define TP2pin A2

//Mapping from sequential drive states to motor phase outputs
/*
State   L1  L2  L3
0       H   -   L
1       -   H   L
2       L   H   -
3       L   -   H
4       -   L   H
5       H   L   -
6       -   -   -
7       -   -   -
*/

////////////////////////////////////////////////////////////////////////////////
////////////////////////////// FUNCTION DECLARATIONS ///////////////////////////
////////////////////////////////////////////////////////////////////////////////

// Set photointerrupter pins for calling motorControlISR
void setISRPhotoInterruptors();

// Set a given drive state
extern void motorOut(int8_t driveState);

// Convert photointerrupter inputs to a rotor state
extern inline int8_t readRotorState();

// Basic synchronisation routine
extern int8_t motorHome();

// Interrupt Routine. Called on every change in encoder input. Runs the motor.
extern void motorControlISR();

// Ticker function. Enables while loop in motorCtrlFn every 100ms using Ticker.
extern void motorCtrlTick();

// Computes velocity and rotational position. Uses PWMController class to set
// motor power based on computed values.
extern void motorCtrlFn();