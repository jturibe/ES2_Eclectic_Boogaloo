#ifndef MESSAGING_H
#define MESSAGING_H

#include "mbed.h"
#include "motor_control.h"
#include "tuner.h"

////////////////////////////////////////////////////////////////////////////////
// Header file containing all declarations of functions and external global   //
// variables that dictate messaging and serial input and output.              //
//                                                                            //
// Global Variables                                                           //
// Functions                                                                  //
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//////////////////////////// GLOBAL VARIABLES //////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

extern volatile float maxVelocity; // Selected VELOCITY by user
extern Mutex maxVelocity_mutex; // Mutex protecting resource selected VELOCITY

extern volatile uint64_t newKey; // Selected KEY by user
extern Mutex newKey_mutex; // Mutex protecting resource selected KEY

extern volatile float selectRotations; // Selected ROTATIONS by user
extern Mutex selectRotations_mutex; // Mutex protecting resource selected ROTATIONS

extern volatile float noteFrequencies[16];
extern Mutex noteFrequencies_mutex;

extern volatile float noteDurations[16];
extern Mutex noteDurations_mutex;

extern volatile int melodyLength;
extern Mutex melodyLength_mutex;

extern Thread tunerThread;

////////////////////////////////////////////////////////////////////////////////
//////////////////////////// FUNCTIONS /////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

// Called by output thread out_comms_thread - sends messages in mail
// box queue along serial connection to host
extern void output_thread();

// Helper function that passes C string messages to the mail box queue
extern void putMessage(char* mssg);

// Interrupt service routine that places characters from serial input messages
// into the character queue inCharQ
extern void serialISR();

// Called by input thread in_comms_thread - Receives characters from
// inCharQ and assembles the characters into their original message. Then,
// interprets the message and implements the corresponding input command.
extern void input_thread();

#endif
