#include "current_task.h"
#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"

void current_task(void const *argument)
{
	for (;;)
	{
		osDelay(1);
	}
}
