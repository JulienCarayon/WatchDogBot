#include <global.hpp>
#include <Arduino.h>

string convertToString(char *character_array, int size)
{
    string convert_string = "";
    for (int i = 0; i < size; i++)
    {
        convert_string += character_array[i];
    }
    return convert_string;
}

