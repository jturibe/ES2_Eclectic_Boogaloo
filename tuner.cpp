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
    tuning_mutex.lock();
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
    new_tune = true;
    tuning_mutex.unlock();
}

void playTune(){
    float noteFrequencies_local[16];
    float noteDurations_local[16];
    int melodyLength_local;
    while(1){
        tuning_mutex.lock();
        if(new_tune){
            std::copy(std::begin(noteFrequencies), std::end(noteFrequencies), std::begin(noteFrequencies_local));
            std::copy(std::begin(noteDurations), std::end(noteDurations), std::begin(noteDurations_local));
            melodyLength_local = melodyLength;
            new_tune = false;
        }
        tuning_mutex.unlock();
        for(int i = 0; i < melodyLength_local; i++){
            PWM_PRD_mutex.lock();
            PWM_PRD = (1/noteFrequencies_local[i])*1000000;
            MotorPWM.period_us(PWM_PRD);
            PWM_PRD_mutex.lock();
            // char message[150];
            // sprintf(message, "Set Note: %d, Melody Length: %d, Note Duration: %d\n\r",PWM_PRD, melodyLength_local, noteDurations_local[i]);
            // putMessage(message);
            Thread::wait(noteDurations_local[i]*1000);
        }
    }
}
