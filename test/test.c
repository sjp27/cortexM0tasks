/**
 * @file       test.c
 *
 * @brief      Implements cortex M0 tasks test.
 *
 * @author     Steve Pickford
 * @date       2023
 */
#include <chip.h>
#include <stdio.h>
#include "cortexM0tasks.h"

#define MAX_TASKS 2
#define TICK_PERIOD_MS 125

static void taskA_handler(void);
static void taskB_handler(void);
	
const uint32_t ExtRateIn = 0;
const uint32_t OscRateIn = 12000000;
const uint32_t RTCOscRateIn = 32768;

int main()
{
	SystemCoreClockUpdate();

	cortexM0tasks_init(MAX_TASKS, TICK_PERIOD_MS);
	cortexM0tasks_init_task(&taskA_handler, 640);
	cortexM0tasks_init_task(&taskB_handler, 640);
	
	cortexM0tasks_start();

	/* Should never reach here */
	while(TRUE)
	{
	}
	
	return 0;
}

static void taskA_handler(void)
{
	int count = 0;

	while (TRUE)
	{
		printf("A%d\n", count++);
		cortexM0tasks_sleep_ms(2000);
	}
}

static void taskB_handler(void)
{
	int count = 0;

	while (TRUE)
	{
		printf("B%d\n", count++);
		cortexM0tasks_sleep_ms(1000);
	}
}
