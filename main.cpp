#include "mbed.h"
#include "Timer.h"
#include <stdio.h>

#include "motor_control.h"
#include "mining_bitcoin.h"
#include "messaging.h"

Thread out_comms_thread;
Thread in_comms_thread;
Thread motorCtrlT(osPriorityNormal,1024);


/////////////////////////// Main
int main() {


    Timer t;
    // BitcoinMiner miner;
    // putMessage("Hello\n\r");

    // Set ISR Photointerruptors to call motor control ISR for running motor
    setISRPhotoInterruptors();

    //putMessage("After setISRPhotoInterruptors\n\r");

    out_comms_thread.start(output_thread);
    in_comms_thread.start(input_thread);
    motorCtrlT.start(motorCtrlFn);
    // t.start();
    while (1) {
        // miner.compute_hash();
        // if (t.read() >= 1){
        //     // char message[100];
        //     // sprintf(message, "Current Computation Rate: %d Hashes per second\n\r",hash_count);
        //     // putMessage(message);
        //     // char key_test_message[100];
        //     // sprintf(key_test_message, "Using key: %d\n\r",*key);
        //     // putMessage(key_test_message);
        //     t.reset();
        //     hash_count = 0;
        // }
        // hash_count++;
    }
}
