
#include <stdlib.h>
#include <math.h>
#include "bmx_utilities.h"

void float_to_2ints(float input, int *output, unsigned int dec)
{
  int dec_places = pow(10,dec);
  
  output[0] = (int) input; // number
  output[1] = abs(input*dec_places - output[0]*dec_places); // decimal
}
