/**
 * @file       cortexM0tasks.c
 * @brief      Implements cortex M0 tasks, simple co-operative multitasker.
 * @author     Steve Pickford
 * @date       2023
 */
#include <chip.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "cortexM0tasks.h"

#define STACK_FRAME 16

static void task_finished(void);

/**< Task status */ 
typedef enum 
{
    TASK_SLEEPING,
    TASK_RUNNING,
    TASK_FINISHED
} task_status_t;

/**< Task structure */ 
typedef struct 
{
    volatile uint32_t sp;          /**< Stack pointer */ 
    
    volatile struct
    {
        task_status_t status : 2;  /**< Task status */
        uint32_t sleep : 30;       /**< Sleep counter */
    } t; /**< Task status structure */        
    void (*handler)(void);         /**< Task handler  */
} task_t;

/**< Tasks structure */ 
typedef struct
{
    uint32_t max_tasks;       /**< Max tasks */
    uint32_t tick_period_ms;  /**< Tick period in ms */
    uint32_t num_tasks;       /**< Number of tasks */
    task_t tasks[];           /**< Array of tasks */
} tasks_t;

tasks_t *m_tasks;                /**< Tasks */
volatile task_t *m_current_task; /**< Current task running */
volatile task_t *m_next_task;    /**< Next task to run */

/**
 * @brief  Initialse tasks
 * @param  z_max_tasks       Maximum number of tasks
 * @param  z_tick_period_ms  Tick period in milliseconds
 */
void cortexM0tasks_init(uint32_t z_max_tasks, uint32_t z_tick_period_ms)
{
    m_tasks = (tasks_t *)malloc(sizeof(tasks_t) + sizeof(task_t) * z_max_tasks);
    m_tasks->max_tasks = z_max_tasks;
    m_tasks->tick_period_ms = z_tick_period_ms;
    m_tasks->num_tasks = 0;
}

/**
 * @brief  Initialise a task
 * @param  z_handler     Task handler
 * @param  z_stack_size  Stack size in bytes
 * @return SUCCESS or ERROR
 */
Status cortexM0tasks_init_task(void (*z_handler)(void), uint32_t z_stack_size)
{
    if (m_tasks->num_tasks < m_tasks->max_tasks)
    {
        uint32_t stack_size = z_stack_size/sizeof(uint32_t);
        uint32_t *stack = NULL;

        /* Initialize the task setting SP to the top of the stack
           minus the STACK_FRAME used for saved registers */
        stack = (uint32_t *)malloc(sizeof(uint32_t)*stack_size);
        if(stack == NULL)
        {
            return ERROR;
        }
        for(int i = 0; i < stack_size; i++)
        {
            stack[i] = 0xA5A5A5A5;
        }
        
        task_t *p_task  = &m_tasks->tasks[m_tasks->num_tasks];
        p_task->sp      = (uint32_t)(stack+stack_size-STACK_FRAME);
        p_task->t.sleep  = 0;
        p_task->t.status = TASK_SLEEPING;
        p_task->handler = z_handler;

        /* Save registers which will be restored on EXC_RETURN
           XPSR: Default value (0x01000000)
           PC: Task handler
           LR: Function to be called when the task handler returns */
        stack[stack_size - 1] = 0x01000000;
        stack[stack_size - 2] = (uint32_t)z_handler;
        stack[stack_size - 3] = (uint32_t)&task_finished;
        m_tasks->num_tasks++;

        return SUCCESS;
    }
    else
    {
        return ERROR;
    }
}

/**
 * @brief  Start tasks, initialises PendSV and SysTick interrupts
 *
 * @return SUCCESS or ERROR
 */
Status cortexM0tasks_start()
{
    NVIC_SetPriority(PendSV_IRQn, 0xFF);  /* Lowest priority */
    NVIC_SetPriority(SysTick_IRQn, 0x00); /* Highest priority */

    /* Start the SysTick timer: */
    uint32_t ret = SysTick_Config(SystemCoreClock/(1000/m_tasks->tick_period_ms));
    if (ret != 0)
    {
        return ERROR;
    }

    /* Start the first task: */
    m_current_task = &m_tasks->tasks[0];
    m_current_task->t.status = TASK_RUNNING;

    __set_PSP(m_current_task->sp+64); /* Set PSP to the top of task's stack */
    __set_CONTROL(0x03); /* Switch to PSP, unprivileged mode */
    __ISB(); /* ISB after changing CONTROL */

    m_current_task->handler();

    return SUCCESS;
}

/**
 * @brief  Task sleep
 *
 * @param  z_sleep_ms  Sleep milliseconds
 */
void cortexM0tasks_sleep_ms(uint32_t z_sleep_ms)
{
    bool task_running = false;
    
    if(m_current_task->t.status != TASK_FINISHED)
    {
        m_current_task->t.status = TASK_SLEEPING;
        __disable_irq();
        m_current_task->t.sleep  = z_sleep_ms/m_tasks->tick_period_ms;
        __enable_irq();
    }

    do
    {
        for(int task = 0; task < m_tasks->num_tasks; task++)
        {
            if(m_tasks->tasks[task].t.status != TASK_FINISHED)
            {
                __disable_irq();
                uint32_t sleep = m_tasks->tasks[task].t.sleep;
                __enable_irq();

                if(sleep == 0)
                {
                    m_next_task = &m_tasks->tasks[task];
                    m_next_task->t.status = TASK_RUNNING;
                    task_running = true;
                    break;
                }
            }
        } 
    } while(task_running == false);   

    /* Trigger PendSV to perform the context switch */
    SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;
    __ISB();
}

/**
 * @brief  SysTick handler
 */
void SysTick_Handler(void)
{
    for(int task = 0; task < m_tasks->num_tasks; task++)
    {
        if(m_tasks->tasks[task].t.sleep > 0)
        {
            m_tasks->tasks[task].t.sleep--;
        }
    }    
}

/**
 * @brief  PendSV_Handler
 */
void __attribute__((naked, noreturn)) PendSV_Handler()
{
	/* Disable interrupts */
	asm("cpsid i");
	/* Save registers R4-R11 */
	asm("mrs r0, psp");
	asm("sub r0, #16");
	asm("stmia r0!,{r4-r7}");
	asm("mov r4, r8");
	asm("mov r5, r9");
	asm("mov r6, r10");
	asm("mov r7, r11");
	asm("sub r0, #32");
	asm("stmia r0!,{r4-r7}");
	asm("sub r0, #16");
	/* Save current tasks SP: */
	asm("ldr r2, =m_current_task");
	asm("mov r3, r2");
	asm("ldr r1, [r2]");
	asm("str r0, [r1]");
	/* Load next tasks SP: */
	asm("ldr r2, =m_next_task");
	asm("ldr r1, [r2]");
	asm("ldr r0, [r1]");
	/* m_current_task = m_next_task: */
	asm("str r1, [r3]");
	/* Restore registers R4-R11 */
	asm("ldmia r0!,{r4-r7}");
	asm("mov r8, r4");
	asm("mov r9, r5");
	asm("mov r10, r6");
	asm("mov r11, r7");
	asm("ldmia r0!,{r4-r7}");
	asm("msr psp, r0");
	/* EXC_RETURN thread mode with PSP */
	asm("ldr r0, =0xFFFFFFFD");
	/* Enable interrupts: */
	asm("cpsie i");
	asm("bx	r0");
}

/**
 * @brief  Called when task handler returns
 */
static void task_finished(void)
{
    m_current_task->t.status = TASK_FINISHED;
    cortexM0tasks_sleep_ms(0);
}
