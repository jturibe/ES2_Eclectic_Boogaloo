#include "tuner.h"

std::map<std::string, float> frequencies = {
    { "C", 261.63 }, { "C#", 277.18 },
    { "D^", 277.18 }, { "D", 293.66 }, { "D#", 311.13 },
    { "E^", 311.13 }, { "E", 329.63 },
    { "F", 349.23 }, { "F#", 369.99 },
    { "G^", 369.99 }, { "G", 392.00 }, { "G#", 415.30 },
    { "A^", 415.30 }, { "A", 440.00 }, { "A#", 466.16 },
    { "B^", 466.16 }, { "B", 493.88 }
};

void tune_parser(std::string melody) {
    int melody_length = melody.size();
    // Go through each character of the string and separate on numbers
    // Put each substring in a vector
    int end_of_note = 0;
    int start_of_note = 0;
    int duration;
    int note_num = 0;
    while(end_of_note <= (melody_length - 1)){
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
}

void playTune(){
   // Timer t;
   // t.start();
    while(1){
        for(int i = 0; i < melodyLength; i++){
            //t.reset();
            PWM_PRD = (1/noteFrequencies[i])*1000000;
            MotorPWM.period_us(PWM_PRD);
            char message[150];
            sprintf(message, "Set Note: %d, Melody Length: %d, Note Duration: %d\n\r",PWM_PRD, melodyLength, noteDurations[i]);
            putMessage(message);
//            while(t.read()<noteDurations[i]){
//                continue;
//            }
            Thread::wait(noteDurations[i]*1000);
        }
    }
}
