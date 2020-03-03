#include "mbed.h"
#include "SHA256.h"
#include "Timer.h"

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
const int8_t lead = 2;  //2 for forwards, -2 for backwards

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
uint8_t hash_result[32];
Timer t;

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

void check_motor_output_flow(){
    static int8_t intStateOld;
    int8_t intState = readRotorState();
    motorOut((intState-orState+lead+6)%6); //+6 to make sure the remainder is positive
    if (intState == 4 && intStateOld == 3) TP1 = !TP1;
    intStateOld = intState;
}

/////////////////////////// Main
int main() {
    orState = 0;    //Rotor offset at motor state 0
    int hash_count = 0;
    SHA256 algo;
    const int32_t PWM_PRD = 2500;
    MotorPWM.period_us(PWM_PRD);
    MotorPWM.pulsewidth_us(PWM_PRD);
    //Initialise the serial port
    Serial pc(SERIAL_TX, SERIAL_RX);
    pc.printf("Hello\n\r");

    //Run the motor synchronisation
    orState = motorHome();
    pc.printf("Rotor origin: %x\n\r",orState);
    //orState is subtracted from future rotor state inputs to align rotor and motor states

    MotorPWM.pulsewidth_us(PWM_PRD/2);
    check_motor_output_flow(); //Try and start the motor without need to spin it
    // ------------------------------------------------------------------------------------------------------------
    // I have no idea what's going on bois
    I1.rise(&check_motor_output_flow);
    I1.fall(&check_motor_output_flow);
    I2.rise(&check_motor_output_flow);
    I2.fall(&check_motor_output_flow);
    I3.rise(&check_motor_output_flow);
    I3.fall(&check_motor_output_flow);
    // ------------------------------------------------------------------------------------------------------------

    //Poll the rotor state and set the motor outputs accordingly to spin the motor
    t.start();
    pc.printf("AFTER T.START\n");
    while (1) {
        algo.computeHash(hash_result, sequence, 64);
        pc.printf("AFTER compute hash\n");
        if ((hash_result[0]==0) && (hash_result[1]==0)) {
            pc.printf("Sucessful nonce, Integer rep: %d \n", *nonce);
            pc.printf("Successful nonce, Hex rep: 0x%X \n", *nonce);
            hash_count++;
        }
        pc.printf("AFTER hash result\n");
        if (t.read() >= 1){
            pc.printf("Current Computation Rate: %d Hashes per second\n",hash_count);
            t.reset();
            hash_count = 0;
        }
        pc.printf("AFTER time eval\n");
        (*nonce)++;
        pc.printf("AFTER nonce increment\n");
    }
}
