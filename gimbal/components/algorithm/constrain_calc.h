#ifndef __CONSTRAIN_CALC_H
#define __CONSTRAIN_CALC_H
#include "struct_typedef.h" 
void ConstrainLoop(float* output, float input, float min, float max);
void SectionTransform(float* output, float input, float out_min, float out_zero, float out_max, float in_min, float in_zero, float in_max);
void MinMaxConstrain(float* input, float min, float max);
float Abs(float num);
float random_spin_PreventFullCircleConstrain(float set, float fdb);
float PreventFullCircleConstrain(float set, float fdb);
float yaw_PreventFullCircleConstrain(float set, float fdb);
float PreventFullCircleConstrain_ecd(float set, float fdb);

#endif
