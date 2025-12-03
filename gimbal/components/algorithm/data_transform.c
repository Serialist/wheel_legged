
#include "data_transform.h"
float Int16ToFloat(int16_t data)
{
	if (data >= 0)
	{
		return (float)data;
	}
	else
	{
		float temp = (float)-data;
		return -temp;
	}
}

int16_t FLOAT_TO_INT16(float data)
{
if(data>=0)
{
return ((int16_t)(data));
}

if(data<0)
{
return -(int16_t)(ABS(data));
}

}
