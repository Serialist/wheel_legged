#include "constrain_calc.h"

void MinMaxConstrain(float* input, float min, float max)
{
	if(*input > max)
	{
		*input = max;
	}
	else if(*input < min)
	{
		*input = min;
	}
}

