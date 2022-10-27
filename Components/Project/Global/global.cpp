#include "Components/Project/Global/global.hpp"

String convertToString(char *character_array, int size)
{
  String convert_string = "";
  for (int i = 0; i < size; i++)
  {
    convert_string += character_array[i];
  }
  return convert_string;
}