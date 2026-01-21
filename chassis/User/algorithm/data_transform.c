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
