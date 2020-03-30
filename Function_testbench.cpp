#include "mbed.h"
#include "Timer.h"
#include "Crypto.h"
#include <stdio.h>
#include <map>
#include <algorithm>
#include <atomic>
#include <string>
#include <sstream>
#include <iostream>

///////////////////////////////////////////////////////////////////////////////
///////////////////////// Global Variables ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
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

#define TUNER_DURATION_CONST 0.125

// Drive state to output table
const int8_t driveTable[] = {0x12,0x18,0x09,0x21,0x24,0x06,0x00,0x00};

// Mapping from interrupter inputs to sequential rotor states. 0x00 and 0x07 are not valid
const int8_t stateMap[] = {0x07,0x05,0x03,0x04,0x01,0x00,0x02,0x07};
//const int8_t stateMap[] = {0x07,0x01,0x03,0x02,0x05,0x00,0x04,0x07}; //Alternative if phase order of input or drive is reversed

// Phase lead to make motor spin
volatile int8_t lead = 2;  //2 for forwards, -2 for backwards

int32_t PWM_PRD = 2000;
Mutex PWM_PRD_mutex;

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
std::atomic<int32_t> motorPosition = {0};

volatile float velocity;

// Initialise message structure
typedef struct {
  char* pure_mssg;
} mail_t;

// Initialise the serial port
// Serial pc(SERIAL_TX, SERIAL_RX);
RawSerial pc(SERIAL_TX, SERIAL_RX);

// Initialise mail box for messages
Mail<mail_t, 24> mail_box;

// Initialise character queue for input messages from host
Mail<uint8_t, 24> inCharQ;

// Selected VELOCITY by user
std::atomic<float> maxVelocity = {100};

// Selected KEY by user
volatile uint64_t newKey;
// Mutex protecting resource selected KEY
Mutex newKey_mutex;

// Selected ROTATIONS by user
std::atomic<float> selectRotations = {100};

volatile float noteFrequencies[16];

volatile float noteDurations[16];

volatile int melodyLength;

Mutex tuning_mutex;

bool first_tune = false;

bool new_tune = false;

std::map<std::string, float> frequencies = {
    { "C", 261.63 }, { "C#", 277.18 },
    { "D^", 277.18 }, { "D", 293.66 }, { "D#", 311.13 },
    { "E^", 311.13 }, { "E", 329.63 },
    { "F", 349.23 }, { "F#", 369.99 },
    { "G^", 369.99 }, { "G", 392.00 }, { "G#", 415.30 },
    { "A^", 415.30 }, { "A", 440.00 }, { "A#", 466.16 },
    { "B^", 466.16 }, { "B", 493.88 }
};

void putMessage(char* mssg){
    if(!mail_box.full()){
        mail_t *mail = mail_box.alloc();
        mail->pure_mssg = mssg;
        mail_box.put(mail);
    }
}

void putCharacter(char a){
    uint8_t *mail = inCharQ.alloc();
    *mail = a;
    inCharQ.put(mail);
}

class BitcoinMiner{
public:
    uint8_t sequence[64] = {0x45,0x6D,0x62,0x65,0x64,0x64,0x65,0x64,
    0x20,0x53,0x79,0x73,0x74,0x65,0x6D,0x73,
    0x20,0x61,0x72,0x65,0x20,0x66,0x75,0x6E,
    0x20,0x61,0x6E,0x64,0x20,0x64,0x6F,0x20,
    0x61,0x77,0x65,0x73,0x6F,0x6D,0x65,0x20,
    0x74,0x68,0x69,0x6E,0x67,0x73,0x21,0x20,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
    uint64_t* key = (uint64_t*)&sequence[48];
    uint64_t* nonce = (uint64_t*)&sequence[56];
    uint8_t hash[32];
    SHA256 algo;

};

BitcoinMiner bm;

///////////////////////////////////////////////////////////////////////////////
////////////////////// Bitcoin mining task ////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
int hash_count = 0;
void bitcoinMiningTask(){
    newKey_mutex.lock();
    *(bm.key) = newKey;
    newKey_mutex.unlock();
    bm.algo.computeHash(bm.hash, bm.sequence, 64);
    if (((bm.hash)[0]==0) && ((bm.hash)[1]==0)) {
        printf("Successful nonce, Hex rep: 0x%X\n\r", *(bm.nonce));
    }
    (*(bm.nonce))++;
}

///////////////////////////////////////////////////////////////////////////////
////////////////////// Running the motor task /////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
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

void motorControlISRTask(){
    static int8_t intStateOld;
    int8_t intState = readRotorState();

    motorOut((intState-orState+lead+6)%6); //+6 to make sure the remainder is positive
    if(intState - intStateOld == 5){
        motorPosition--;
    } else if (intState - intStateOld == -5){
        motorPosition++;
    } else{
        motorPosition += intState - intStateOld;
    }
    intStateOld = intState;
}

///////////////////////////////////////////////////////////////////////////////
///////////// Controlling motor velocity and position task ////////////////////
///////////////////////////////////////////////////////////////////////////////
template <typename T> int sgnMC(T val) {
    return (T(0) < val) - (val < T(0));
}

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

    /// VELOCITY CONSTANTSbitc
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
    PWMController(){
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
    float setVelocity(float error_term){
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
    float setRotation(float error_term, float time){
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
    float pwmController(){
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

};

// PWM Controller to set required pulsewidth, and hence motor power.
PWMController pwmcontrol;

void motorCtrlFnTask(){
    // Initialise required variables
    int32_t old_position = motorPosition;
    Timer timer;
    int64_t position_change;
    float velocity;
    float error_term;
    int iter = 0;
    float y_s_loc;
    float y_r_loc;
    float old_selectRotations = 0.0;
    float local_motorPosition;

    // Set ticker to call motorCtrlTick every 100 ms
    timer.start();

    //fetch the motor position
    local_motorPosition = motorPosition;

    //fetch the time
    float time = timer.read();

    //fetch the maxVelocity
    int32_t local_maxVelocity = maxVelocity;


    // Calculate current velocity
    position_change = local_motorPosition - old_position;
    velocity = position_change/time/6;
    // Reset timer to measure correct period between motor position readings
    timer.reset();

    // Calculate rotation error and find power
    float rotation_error = selectRotations-(local_motorPosition)/6;
    y_r_loc = pwmcontrol.setRotation(rotation_error, time);

    //Find the sign
    int sign = (y_r_loc < 0)? -1: 1;

    // Calculate velocity error and find power
    float velocity_error = local_maxVelocity*sign - velocity;
    y_s_loc = pwmcontrol.setVelocity(velocity_error);


    // Set the lead
    if(rotation_error>0.5){
        lead = y_s_loc < 0 ? -2:2;
    }else{
        //Set the lead to 0 to stop oscilations
        lead = 0;
    }


    float power = (velocity < 0)? std::max(y_s_loc,y_r_loc) : std::min(y_s_loc,y_r_loc);


    PWM_PRD_mutex.lock();
    int local_PWM_PDR = PWM_PRD;
    PWM_PRD_mutex.unlock();




    //Jumpstarting the rotations to get over 0 input heuristics
    if(selectRotations!=old_selectRotations){
        // Set the velocity
        pwmcontrol.s_err = rotation_error > 0? 2400 : -3200;
        MotorPWM.write(1);
        motorControlISRTask();
    }

    old_position = motorPosition;
    old_selectRotations = selectRotations;
}



///////////////////////////////////////////////////////////////////////////////
//////////////////////////// Output to serial task ////////////////////////////
///////////////////////////////////////////////////////////////////////////////

void output_threadTask(){
    osEvent evt = mail_box.get();
    if (evt.status == osEventMail) {
        mail_t *mail = (mail_t*)evt.value.p;
        printf("%s", mail->pure_mssg);
        mail_box.free(mail);
    }
}

///////////////////////////////////////////////////////////////////////////////
////////////////////// Serial input task //////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

void serialISRTask(){
    if(!inCharQ.full()){
        uint8_t* newChar = inCharQ.alloc();
        *newChar = 'f';
        inCharQ.put(newChar);
    }
}

///////////////////////////////////////////////////////////////////////////////
////////////////////// Input message decoding task ////////////////////////////
///////////////////////////////////////////////////////////////////////////////

void tune_parser(std::string melody) {
    int melody_length = melody.size() - 2;
    // Go through each character of the string and separate on numbers
    // Put each substring in a vector
    int end_of_note = 1;
    int start_of_note = 1;
    int duration;
    int note_num = 0;
    tuning_mutex.lock();
    while(end_of_note <= melody_length){
        char string_element = melody[end_of_note];
        if(string_element > 48 && string_element < 57){
            std::string note = melody.substr(start_of_note,end_of_note-start_of_note);
            noteFrequencies[note_num] = frequencies[note];
            duration = string_element - '0';
            noteDurations[note_num] = duration*TUNER_DURATION_CONST;
            start_of_note = end_of_note + 1;
            note_num++;
        }
        end_of_note++;
    }
    melodyLength = note_num;
    new_tune = true;
    tuning_mutex.unlock();
}

std::string input_for_thread = "TA#8G#8D#8C#8A#8G#8D#8C#8A#8G#8D#8C#8A#8G#8D#8C#8";

void input_threadTask(){
    //pc.attach(&serialISRTask);
    // input message
    // Take in character and add to input message
    osEvent newEvent = inCharQ.get();
    uint8_t* newChar = (uint8_t*)newEvent.value.p;
    input_for_thread.push_back(*newChar);
    // If '\r', end of message reached, time to decode
    if (*newChar == '\r'){
        switch(input_for_thread[0]){
            case 'K':{
                // New Key command, of form: K[0-9a-fA-F]{16}
                newKey_mutex.lock();
                sscanf(input_for_thread.c_str(),"K%x",&newKey);
                newKey_mutex.unlock();
                break;
            }

            case 'V':{
                // Set maximum velocity command, of form: V\d{1,3}(\.\d)?
                sscanf(input_for_thread.c_str(),"V%f",&maxVelocity);
                break;
            }

            case 'R':{
                // Set target rotations command, of form: R-?\d{1,4}(\.\d)?
                float input_rotations;
                sscanf(input_for_thread.c_str(), "R%f", &input_rotations);
                selectRotations = ((float)motorPosition)/6 + input_rotations;
                break;
            }

            case 'T':{
            // Set tune, of form: T([A-G][#^]?[1-8]){1,16} (where # and ^ are characters)
                tune_parser(input_for_thread);
                if(!first_tune){
                    first_tune = true;
                //    tunerThread.start(playTune);
                }
                break;
            }
            // Tester code for receiving PWM torque, unused for spec
            case 't':{
                sscanf(input_for_thread.c_str(),"t%d",&pwmTorque);
                break;
            }

            default:{
                ;
            }

        }
           // Clear characters for next input message
           input_for_thread = "";
    }
    // Free up character queue
    inCharQ.free(newChar);
}

std::string minimum_input_for_thread = "";
void MINIMUM_input_threadTask(){
    //pc.attach(&serialISRTask);
    // input message
    // Take in character and add to input message
    osEvent newEvent = inCharQ.get();
    uint8_t* newChar = (uint8_t*)newEvent.value.p;
    minimum_input_for_thread.push_back(*newChar);
    // If '\r', end of message reached, time to decode
    if (*newChar == '\r'){
        switch(input_for_thread[0]){
            case 'K':{
                // New Key command, of form: K[0-9a-fA-F]{16}
                newKey_mutex.lock();
                sscanf(minimum_input_for_thread.c_str(),"K%x",&newKey);
                newKey_mutex.unlock();
                break;
            }

            case 'V':{
                // Set maximum velocity command, of form: V\d{1,3}(\.\d)?
                sscanf(minimum_input_for_thread.c_str(),"V%f",&maxVelocity);
                break;
            }

            case 'R':{
                // Set target rotations command, of form: R-?\d{1,4}(\.\d)?
                float input_rotations;
                sscanf(minimum_input_for_thread.c_str(), "R%f", &input_rotations);
                selectRotations = ((float)motorPosition)/6 + input_rotations;
                break;
            }

            case 'T':{
            // Set tune, of form: T([A-G][#^]?[1-8]){1,16} (where # and ^ are characters)
                std::string tune_string = minimum_input_for_thread.substr(1, input_for_thread.length() - 1);
                tune_parser(tune_string);
                if(!first_tune){
                    first_tune = true;
                //    tunerThread.start(playTune);
                }
                break;
            }
            // Tester code for receiving PWM torque, unused for spec
            case 't':{
                sscanf(minimum_input_for_thread.c_str(),"t%d",&pwmTorque);
                break;
            }

            default:{
                ;
            }

        }
           // Clear characters for next input message
           minimum_input_for_thread = "";
    }
    // Free up character queue
    inCharQ.free(newChar);
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////// Playing tune task ///////////////////////////////
///////////////////////////////////////////////////////////////////////////////
float noteFrequencies_local[16];
float noteDurations_local[16];
int melodyLength_local;
void playTuneTask(){
   tuning_mutex.lock();
   if(new_tune){
       std::copy(std::begin(noteFrequencies), std::end(noteFrequencies), std::begin(noteFrequencies_local));
       std::copy(std::begin(noteDurations), std::end(noteDurations), std::begin(noteDurations_local));
       melodyLength_local = melodyLength;
       new_tune = false;
   }
   tuning_mutex.unlock();
   PWM_PRD_mutex.lock();
   PWM_PRD = (1/noteFrequencies_local[0])*1000000;
   MotorPWM.period_us(PWM_PRD);
   PWM_PRD_mutex.unlock();
   // char message[150];
   // sprintf(message, "Set Note: %d, Melody Length: %d, Note Duration: %d\n\r",PWM_PRD, melodyLength_local, noteDurations_local[i]);
   // putMessage(message);
}

/////////////////////////// Main
int main() {
    Timer timer;
//    printf("Timing Bitcoin Mining Task...\n\r");
    timer.start();
    // for(int i = 0; i < 5000; i++){
    //     bitcoinMiningTask();
    // }

    float bitcoinTime = timer.read();
    printf("Bitcoin Mining Task time recorded.\n\r");
    //
    // printf("Timing Running The Motor Task...\n\r");
    // MotorPWM.period_us(PWM_PRD);
    // MotorPWM.pulsewidth_us(PWM_PRD);
    // orState = motorHome();
    // timer.reset();
    // for(int i = 0; i < 5000; i++){
    //     motorControlISRTask();
    // }
    //
    // float motorRunTime = timer.read();
    // printf("Running The Motor Task time recorded.\n\r");
    //
    // printf("Timing Controlling Motor velocity and position Task...\n\r");
    // timer.reset();
    // for(int i = 0; i < 5000; i++){
    //     motorCtrlFnTask();
    // }
    //
    // float motorControlTime = timer.read();
    // printf("Controlling Motor velocity and position Task time recorded.\n\r");
    //
    // printf("Timing Output to Serial Task...\n\r");
    // float outputSerialTime = 0;
    // for(int i = 0; i < 5000; i++){
    //     char message[100];
    //     sprintf(message, "Successful nonce, Hex rep: 0x%X\n\r", *(bm.nonce));
    //     putMessage(message);
    //     timer.reset();
    //     output_threadTask();
    //     outputSerialTime += timer.read();
    // }
    //
    // printf("Output to Serial Task time recorded.\n\r");
    //
    // printf("Timing Serial Input ISR Task...\n\r");
    // float SerialInputISRTime = 0;
    // for(int i = 0; i < 5000; i++){
    //     timer.reset();
    //     serialISRTask();
    //     SerialInputISRTime += timer.read();
    //     osEvent newEvent = inCharQ.get();
    //     uint8_t* newChar = (uint8_t*)newEvent.value.p;
    //     inCharQ.free(newChar);
    // }
    //
    // printf("Serial Input ISR Task time recorded.\n\r");

    printf("Timing Input Decoder Task...\n\r");
    float InputdecodingTime = 0;
    for(int i = 0; i < 5000; i++){
        putCharacter('\r');
        timer.reset();
        input_threadTask();
        InputdecodingTime += timer.read();
        input_for_thread = "TA#8G#8D#8C#8A#8G#8D#8C#8A#8G#8D#8C#8A#8G#8D#8C#8";
    }

    // printf("Input Decoder Task time recorded.\n\r");
    //
    // printf("Timing Playing Tune Task...\n\r");
    // timer.reset();
    // for(int i = 0; i < 5000; i++){
    //     playTuneTask();
    // }
    //
    // float PlayingtuneTime = timer.read();
    // printf("Playing Tune Task time recorded.\n\r");
    //
     printf("Bitcoin mining time: %f\n\r", bitcoinTime);
    // printf("Running the motor time: %f\n\r", motorRunTime/5000);
    // printf("Controlling motor vel & pos time: %f\n\r", motorControlTime/5000);
    // printf("Output to serial Time: %f\n\r", outputSerialTime/5000);
    // printf("Serial Input ISR Time: %f\n\r", SerialInputISRTime/5000);
    printf("Input decoder Time: %f\n\r", InputdecodingTime/5000);
    // printf("Playing a tune time: %f\n\r", PlayingtuneTime/5000);
    //
    //
    // printf("FINDING MINIMUM EXECUTION TIME OF INPUT DECODING THREAD\n\r");
    // float MinimumInputdecodingTime = 0;
    // for(int i = 0; i < 5000; i++){
    //     putCharacter('V');
    //     timer.reset();
    //     MINIMUM_input_threadTask();
    //     MinimumInputdecodingTime += timer.read();
    //     minimum_input_for_thread = "";
    // }
    //
    // printf("MINIMUM Input decoder Time: %f\n\r", MinimumInputdecodingTime/5000);
}
