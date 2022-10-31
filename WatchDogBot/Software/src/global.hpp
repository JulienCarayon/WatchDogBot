#ifndef GLOBAL_H
#define GLOBAL_H

#include <Arduino.h>
#include <string>
#include <stdio.h>
#include <stdlib.h>

using namespace std;

typedef struct
{
    float distance_right;
    float distance_left;
} HC_SR04_DATA_T;

typedef enum
{
    IDLE = 0,
    MANUAL = 1,
    GUARDING = 2,
    DETECTED = 3,
    ERROR = 4
} FSM_T;

typedef struct
{
    bool idleState;
    bool manualState;
    bool guardingState;
    bool detectedState;
    bool errorState;
} g_fsm_flags;

struct CALLBACK
{
    bool new_message;
    string topic;
    string message;
    FSM_T fsm_state;
};

string convertToString(char *character_array, int size);

#endif