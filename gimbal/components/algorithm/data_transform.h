#ifndef __DATA_TRANSFORM_H
#define __DATA_TRANSFORM_H
#include "struct_typedef.h"
#define ABS(x) ((x)>0? (x):(-(x))) 
float Int16ToFloat(int16_t data);
int16_t FLOAT_TO_INT16(float data);
#endif
