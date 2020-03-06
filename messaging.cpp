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
volatile float maxVelocity;
// Mutex protecting resource selected VELOCITY
Mutex maxVelocity_mutex;

// Selected KEY by user
volatile uint64_t newKey;
// Mutex protecting resource selected KEY
Mutex newKey_mutex;

// Selected ROTATIONS by user
volatile float selectRotations;
// Mutex protecting resource selected ROTATIONS
Mutex selectRotations_mutex;

////////////////////////////////////////////////////////////////////////////////
//////////////////////////// FUNCTIONS /////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

//// FUNCTION: Called by output thread out_comms_thread - sends messages in mail
//// box queue along serial connection to host
void output_thread() {
    while(true){
        osEvent evt = mail_box.get(1);
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
        osEvent newEvent = inCharQ.get(1);
        uint8_t* newChar = (uint8_t*)newEvent.value.p;
        input.push_back(*newChar);
        // If '\r', end of message reached, time to decode
        if (*newChar == '\r'){
            putMessage("GIMME GIMME GIMME \n\r");
            switch(input[0]){
                case 'K':
                // New Key command, of form: K[0-9a-fA-F]{16}
                    newKey_mutex.lock();
                    sscanf(input.c_str(),"K%x",&newKey);
                    newKey_mutex.unlock();
                    break;

                case 'V':
                // Set maximum velocity command, of form: V\d{1,3}(\.\d)?
                    maxVelocity_mutex.lock();
                    sscanf(input.c_str(),"V%f",&maxVelocity);
                    maxVelocity_mutex.unlock();
                    break;

                case 'R':
               // Set target rotations command, of form: R-?\d{1,4}(\.\d)?
                    float input_rotations;
                    sscanf(input.c_str(), "R%f", &input_rotations);
                    selectRotations_mutex.lock();
                    selectRotations = ((float)motorPosition)/6 + input_rotations;
                    selectRotations_mutex.unlock();
                    break;
              // Tester code for receiving PWM torque, unused for spec
                case 't':
                    sscanf(input.c_str(),"t%d",&pwmTorque);
                    break;
                default:
                ;
            }
            // Clear characters for next input message
            input = "";
        }
        // Free up character queue
        inCharQ.free(newChar);
    }
}
