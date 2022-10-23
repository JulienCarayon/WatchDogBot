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

string convertToString(char *character_array, int size);

#endif