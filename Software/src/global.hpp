#ifndef GLOBAL_H
#define GLOBAL_H

#include <Arduino.h>
#include <string>
#include <stdio.h>
#include <stdlib.h>

using namespace std;

struct CALLBACK
{
    bool new_message;
    string topic;
    string message;
};

struct HC_SR04_DATA_T
{
    float distance_right;
    float distance_left;
};

string convertToString(char *character_array, int size);

#endif