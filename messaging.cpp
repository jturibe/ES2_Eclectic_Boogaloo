#include "messaging.h"

////////////////////////////////////////////////////////////////////////////////
// GLOBAL VARIABLES and FUNCTIONS used specifically for controlling the motor.//
// See header file for FUNCTION DECLARATIONS and PIN DEFINITIONS.             //
// FUNCTIONS in file:                                                         //
// - output_thread()                                                          //
// - putMessage()                                                             //
// - serialISR()                                                              //
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//////////////////////////// GLOBAL VARIABLES //////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

// Initialise message structure
typedef struct {
  char* pure_mssg;
} mail_t;

// Initialise the serial port
// Serial pc(SERIAL_TX, SERIAL_RX);
RawSerial pc(SERIAL_TX, SERIAL_RX);

// Initialise mail box for messages
Mail<mail_t, 16> mail_box;

// Initialise character queue for input messages from host
Mail<uint8_t, 24> inCharQ;

// Selected VELOCITY by user
std::atomic<float> maxVelocity = {100};

// Selected KEY by user
volatile uint64_t newKey;
// Mutex protecting resource selected KEY
Mutex newKey_mutex;

// Selected ROTATIONS by user
std::atomic<float> selectRotations;

volatile float noteFrequencies[16];

volatile float noteDurations[16];

volatile int melodyLength;

Mutex tuning_mutex;

bool first_tune = false;

bool new_tune = false;

////////////////////////////////////////////////////////////////////////////////
//////////////////////////// FUNCTIONS /////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

//// FUNCTION: Called by output thread out_comms_thread - sends messages in mail
//// box queue along serial connection to host
void output_thread() {
    while(true){
        osEvent evt = mail_box.get();
        if (evt.status == osEventMail) {
            mail_t *mail = (mail_t*)evt.value.p;
            printf("%s", mail->pure_mssg);
            mail_box.free(mail);
        }
    }
}

//// FUNCTION: Helper function that passes C string messages to mail box queue
void putMessage(char* mssg){
    if(!mail_box.full()){
        mail_t *mail = mail_box.alloc();
        mail->pure_mssg = mssg;
        mail_box.put(mail);
    }
}

//// FUNCTION: Interrupt service routine that places characters from serial
//// input messages into the character queue inCharQ
void serialISR(){
    if(!inCharQ.full()){
        uint8_t* newChar = inCharQ.alloc();
        *newChar = pc.getc();
        inCharQ.put(newChar);
    }
 }

 //// FUNCTION: Called by input thread in_comms_thread - Receives characters
 //// from inCharQ and assembles the characters into their original message.
 //// Then, interprets the message and implements the corresponding command
void input_thread(){
    pc.attach(&serialISR);
    // input message
    std::string input = "";
    while(1) {
        // Take in character and add to input message
        osEvent newEvent = inCharQ.get();
        uint8_t* newChar = (uint8_t*)newEvent.value.p;
        input.push_back(*newChar);
        // If '\r', end of message reached, time to decode
        if (*newChar == '\r'){
            char message[80];
            sprintf(message, "Received a command, decoding... %s\n\r",input.c_str());
            putMessage(message);
            switch(input[0]){
                case 'K':{
                    // New Key command, of form: K[0-9a-fA-F]{16}
                    newKey_mutex.lock();
                    sscanf(input.c_str(),"K%x",&newKey);
                    newKey_mutex.unlock();
                    break;
                }

                case 'V':{
                    // Set maximum velocity command, of form: V\d{1,3}(\.\d)?
                    sscanf(input.c_str(),"V%f",&maxVelocity);
                    break;
                }

                case 'R':{
                    // Set target rotations command, of form: R-?\d{1,4}(\.\d)?
                    float input_rotations;
                    sscanf(input.c_str(), "R%f", &input_rotations);
                    selectRotations = ((float)motorPosition)/6 + input_rotations;
                    break;
                }

                case 'T':{
                // Set tune, of form: T([A-G][#^]?[1-8]){1,16} (where # and ^ are characters)
                    std::string tune_string = input.substr(1, input.length() - 1);
                    tune_parser(tune_string);
                    if(!first_tune){
                        first_tune = true;
                        tunerThread.start(playTune);
                    }
                    break;
                }
                // Tester code for receiving PWM torque, unused for spec
                case 't':{
                    sscanf(input.c_str(),"t%d",&pwmTorque);
                    break;
                }

                default:{
                    continue;
                }

            }
               // Clear characters for next input message
               input = "";
        }
        // Free up character queue
        inCharQ.free(newChar);
   }
}
