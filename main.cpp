#include "mbed.h"
#include "Crypto.h"
#include "Timer.h"
#include <vector>
#include <stdio.h>
#include <mutex>          // std::mutex
#include <string>

//Photointerrupter input pins
#define I1pin D3
#define I2pin D6
#define I3pin D5

//Incremental encoder input pins
#define CHApin   D12
#define CHBpin   D11

//Motor Drive output pins   //Mask in output byte
#define L1Lpin D1           //0x01
#define L1Hpin A3           //0x02
#define L2Lpin D0           //0x04
#define L2Hpin A6          //0x08
#define L3Lpin D10           //0x10
#define L3Hpin D2          //0x20

#define PWMpin D9

//Motor current sense
#define MCSPpin   A1
#define MCSNpin   A0

//Test outputs
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
//Drive state to output table
const int8_t driveTable[] = {0x12,0x18,0x09,0x21,0x24,0x06,0x00,0x00};

//Mapping from interrupter inputs to sequential rotor states. 0x00 and 0x07 are not valid
const int8_t stateMap[] = {0x07,0x05,0x03,0x04,0x01,0x00,0x02,0x07};
//const int8_t stateMap[] = {0x07,0x01,0x03,0x02,0x05,0x00,0x04,0x07}; //Alternative if phase order of input or drive is reversed

//Phase lead to make motor spin
volatile int8_t lead = 2;  //2 for forwards, -2 for backwards
const int32_t PWM_PRD = 2000;

int8_t orState;

//Status LED
DigitalOut led1(LED1);

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

DigitalOut TP1(TP1pin);
PwmOut MotorPWM(PWMpin);
volatile uint64_t pwmTorque;
int64_t motorPosition = 0;

Thread out_comms_thread;
Thread in_comms_thread;
Thread motorCtrlT(osPriorityNormal,1024);

volatile uint64_t newKey;
Mutex newKey_mutex;

volatile float select_rotations = 0.0;

typedef struct {
  char* pure_mssg;
} mail_t;

//Initialise the serial port
// Serial pc(SERIAL_TX, SERIAL_RX);
RawSerial pc(SERIAL_TX, SERIAL_RX);

Mail<mail_t, 16> mail_box;
Mail<uint8_t, 24> inCharQ;

void motorControlISR();

void putMessage(char* mssg){
    if(!mail_box.full()){
        mail_t *mail = mail_box.alloc();
        mail->pure_mssg = mssg;
        mail_box.put(mail);
    }
}

Mutex maxVelocity_mutex;

class PWMController{
public:
    float y_s; // Final PWM controll value = y_ps + y_is + y_ds
    float y_ps; // Proportional velocity term
    float y_is; // Integral rotation term
    float y_ds; // Differential rotation term

    float y_r; // Final PWM controll value = y_p + y_dr
    float y_pr; // Proportional rotation term
    float y_dr; // Differential rotation term

    float k_ps = 50;
    float k_is = 0.3;
    float k_ds = 0.2;

    float k_pr = 50;
    float k_ir = 0.3;
    float k_dr = 1;

    float c_err; //Cumulative error
    float past_rota_err; // past rotation error
    float maxVelocity;
    float y_is_limit = 1600;

    PWMController(){
        y_s = PWM_PRD/2;
        y_r = PWM_PRD/2;

        y_ps = 0;
        y_is = 0;
        y_ds = 0;

        y_pr = 0;
        y_dr = 0;
        maxVelocity = 100;
        c_err = 0;
        past_rota_err = 0;
    }

    void setVelocity(float error_term){
        //Calculate the proportional term
        y_ps = k_ps*error_term;

        //Calculate the integral term
        c_err += error_term;
        y_is = c_err*k_is;

        y_is = y_is>y_is_limit ? y_is_limit:y_is;
//        y_i = y_i<-y_i_limit ?-y_i_limit:y_i;

        //Calculate PWM controll
        y_s = y_ps + y_is; //+ y_ds

        lead = y_s < 0 ? -2:2;
        y_s = abs(y_s) > PWM_PRD ? PWM_PRD : y_s;

        MotorPWM.pulsewidth_us(abs(y_s));
    }

    void setRotation(float error_term){
        // Calculate the proportional term
        y_pr = k_pr*error_term;

        // Calculate the differential term
        y_dr = k_dr*(error_term - past_rota_err);

        // Calculate PWM control
        y_r = y_pr + y_dr;

        lead = y_r < 0 ? -2:2;
        y_r = abs(y_r) > PWM_PRD ? PWM_PRD : y_r;

        MotorPWM.pulsewidth_us((int)abs(y_r));
        past_rota_err = error_term;
    }
};

PWMController pwmcontroll;

void serialISR(){
    if(!inCharQ.full()){
        uint8_t* newChar = inCharQ.alloc();
        *newChar = pc.getc();
        inCharQ.put(newChar);
    }
 }

void input_thread(){
    pc.attach(&serialISR);
    std::string input = "";
    while(1) {
        osEvent newEvent = inCharQ.get();
        uint8_t* newChar = (uint8_t*)newEvent.value.p;
        input.push_back(*newChar);
        if (*newChar == '\r'){
            switch(input[0]){
                case 'K':
                    newKey_mutex.lock();
                    sscanf(input.c_str(),"K%x",&newKey);
                    newKey_mutex.unlock();
                    break;

                case 'V':
                    maxVelocity_mutex.lock();
                    sscanf(input.c_str(),"V%f",&(pwmcontroll.maxVelocity));
                    maxVelocity_mutex.unlock();
                    break;

                case 'R':
                    float input_rotations;
                    sscanf(input.c_str(), "R%f", &input_rotations);
                    select_rotations = ((float)motorPosition)/6 + input_rotations;
                    break;

                case 't':
                    sscanf(input.c_str(),"t%d",&pwmTorque);
                    break;
                default:
                ;
            }
            input = "";
        }
        inCharQ.free(newChar);
    }
}

// Set a given drive state
void motorOut(int8_t driveState){
    // Lookup the output byte from the drive state.
    int8_t driveOut = driveTable[driveState & 0x07];

    // Turn off first
    if (~driveOut & 0x01) L1L = 0;
    if (~driveOut & 0x02) L1H = 1;
    if (~driveOut & 0x04) L2L = 0;
    if (~driveOut & 0x08) L2H = 1;
    if (~driveOut & 0x10) L3L = 0;
    if (~driveOut & 0x20) L3H = 1;

    // Then turn on
    if (driveOut & 0x01) L1L = 1;
    if (driveOut & 0x02) L1H = 0;
    if (driveOut & 0x04) L2L = 1;
    if (driveOut & 0x08) L2H = 0;
    if (driveOut & 0x10) L3L = 1;
    if (driveOut & 0x20) L3H = 0;
}

    // Convert photointerrupter inputs to a rotor state
inline int8_t readRotorState(){
    return stateMap[I1 + 2*I2 + 4*I3];
}

// Basic synchronisation routine
int8_t motorHome() {
    // Put the motor in drive state 0 and wait for it to stabilise
    motorOut(0);
    wait(2.0);

    // Get the rotor state
    return readRotorState();
}

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

/////////////////////////////Communication
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

void motorCtrlTick(){
    motorCtrlT.signal_set(0x1);
}

void motorCtrlFn(){
    int64_t old_position = motorPosition;
    Ticker motorCtrlTicker;
    Timer timer;
    float mult;
    float position_change;
    float velocity;
    float rotation_error;
    float error_term;
    int iter = 0;
    motorCtrlTicker.attach_us(&motorCtrlTick,100000);
    // MotorPWM.pulsewidth_us(2000);
    timer.start();
    while(1){
        motorCtrlT.signal_wait(0x1);
        position_change = motorPosition - old_position;
        velocity = position_change/timer.read()/6;
        timer.reset();
        // rotation_error = select_rotations - ((float)motorPosition)/6;
        // pwmcontroll.setRotation(rotation_error);
        // maxVelocity_mutex.lock();
        error_term = (pwmcontroll.maxVelocity - velocity);
        pwmcontroll.setVelocity(error_term);
        if(iter == 9){
             char message[150];
             sprintf(message,"MaxVelocity: %f, Motor Velocity: %f, Motor Power: %f\n\r",pwmcontroll.maxVelocity,velocity,pwmcontroll.y_s);
             //sprintf(message, "Motor Velocity: %f, Motor Position: %f, Selected Position: %f\n\r",velocity,((float)motorPosition)/6,select_rotations);
             putMessage(message);
            // char message2[150];
            // sprintf(message2, "Position error: %f, Proportional term: %f, Full term: %f, Lead: %d\n\r",rotation_error, pwmcontroll.y_pr, pwmcontroll.y_r, lead);
            // putMessage(message2);
            // char message3[50];
            // sprintf(message3, "PWM: %f\n\r",MotorPWM.read());
            // putMessage(message3);

            iter = 0;
        }
        // maxVelocity_mutex.unlock();
        iter++;
        old_position = motorPosition;
    }
}

/////////////////////////// Main
int main() {
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
    Timer t;
    int hash_count = 0;
    SHA256 algo;
    MotorPWM.period_us(PWM_PRD);
    MotorPWM.pulsewidth_us(PWM_PRD/2);

    putMessage("Hello\n\r");

    //Run the motor synchronisation
    orState = motorHome();
    char message[100];
    sprintf(message,"Rotor origin: %x\n\r",orState);
    putMessage(message);
    //orState is subtracted from future rotor state inputs to align rotor and motor states
    motorControlISR();
    // ------------------------------------------------------------------------------------------------------------
    // I have no idea what's going on bois
    I1.rise(&motorControlISR);
    I1.fall(&motorControlISR);
    I2.rise(&motorControlISR);
    I2.fall(&motorControlISR);
    I3.rise(&motorControlISR);
    I3.fall(&motorControlISR);
    // ------------------------------------------------------------------------------------------------------------
    out_comms_thread.start(output_thread);
    in_comms_thread.start(input_thread);
    motorCtrlT.start(motorCtrlFn);
    // t.start();
    while (1) {
        putMessage("Hello\n\r");
        newKey_mutex.lock();
        *key = newKey;
        newKey_mutex.unlock();
        algo.computeHash(hash, sequence, 64);
        hash_count++;
        if ((hash[0]==0) && (hash[1]==0)) {
            // char message[100];
            // sprintf(message, "Successful nonce, Hex rep: 0x%X\n\r", *nonce);
            // putMessage(message);
        }
        if (t.read() >= 1){
            // char message[100];
            // sprintf(message, "Current Computation Rate: %d Hashes per second\n\r",hash_count);
            // putMessage(message);
            // char message_debug[100];
            // sprintf(message_debug, "Power set:%d\n\rTarget Velocity: %f\n\r\n\r\n\rs",y_s, maxVelocity);
            // putMessage(message_debug);
            // char key_test_message[100];
            // sprintf(key_test_message, "Using key: %d\n\r",*key);
            // putMessage(key_test_message);
            // char torque_test_message[100];
            // sprintf(torque_test_message, "Using torque: %d\n\r",pwmTorque);
            // putMessage(torque_test_message);
            t.reset();
            hash_count = 0;
        }
        (*nonce)++;
    }
}
