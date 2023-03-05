/**
 * @file       cortexM0tasks.h
 * @brief      Header file for cortex M0 tasks.
 * @author     Steve Pickford
 * @date       2023
 */
#ifndef CORTEXM0TASKS_H
#define CORTEXM0TASKS_H

void cortexM0tasks_init(uint32_t z_max_tasks, uint32_t z_tick_period_ms);
Status cortexM0tasks_init_task(void (*handler)(void), uint32_t z_stack_size);
Status cortexM0tasks_start(void);
void cortexM0tasks_sleep_ms(uint32_t z_sleep_ms);

#endif