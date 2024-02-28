//*****************************************************************************
// File:    trigger_ADC_task.c
// Authors: Luke Peters and Philip Stenger
// Date:    07/08/2023
// Group:   19
// Brief:   The Trigger ADC Task triggers the ADC interrupt. The ISR then sends an ADC
//          sample over a queue to the Altitude Task.
//*****************************************************************************

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/adc.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "utils/uartstdio.h"
#include "priorities.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

//*****************************************************************************
//
// Definitions and Globals
//
//*****************************************************************************

// Trigger ADC Task Parameters
#define TRIGGER_ADC_TASK_STACK_SIZE     128         // Stack size in words
#define TRIGGER_ADC_TASK_DELAY_MS       15          // Trigger ADC delay in milliseconds

// The item size and queue size for the ADC reading queue.
#define ADC_ITEM_SIZE                   sizeof(uint32_t)
#define ADC_QUEUE_SIZE                  5

// Queues
xQueueHandle g_pADCQueue;

//*****************************************************************************
//
// Functions
//
//*****************************************************************************

/**
 * @brief Handles the ADC interrupt, taking an ADC sample and sending it to a queue.
 */
void
ADCISRHandler(void)
{
    uint32_t ui32Altitude;

    // Gets a sample from ADC0.
    ADCSequenceDataGet(ADC0_BASE, 0, &ui32Altitude);

    // Sends the sample to the ADC queue.
    xQueueSendFromISR(g_pADCQueue, &ui32Altitude, NULL);

    ADCIntClear(ADC0_BASE, 0);
}

/**
 * @brief Initialize the ADC and ADC interrupt.
 */
uint32_t
ADCIntInit (void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);

    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0))
    {
    }

    ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_CH9 | ADC_CTL_IE |
                             ADC_CTL_END);

    ADCSequenceEnable(ADC0_BASE, 0);

    // Register the interrupt handler.
    ADCIntRegister (ADC0_BASE, 0, ADCISRHandler);

    // Must set the priority of the interrupt to be less than the specified
    // FreeRTOS interrupts. Use hw_inc.h to find the correct INT value.
    IntPrioritySet(INT_ADC0SS0_TM4C123, pdTM4C_RTOS_INTERRUPT_PRIORITY(7));

    // Enable interrupts for ADC0 sequence 0.
    ADCIntEnable(ADC0_BASE, 0);

    return (0);
}


/**
 * @brief Triggers the ADC interrupt.
 */
static void
TriggerADCTask(void *pvParameters)
{
    portTickType ui32WakeTime = xTaskGetTickCount();  // Initialize ui32WakeTime
    while(1)
    {
        ADCProcessorTrigger(ADC0_BASE, 0);
        vTaskDelayUntil(&ui32WakeTime, TRIGGER_ADC_TASK_DELAY_MS / portTICK_RATE_MS);
    }
}

/**
 * @brief Initializes the Trigger ADC task.
 */
uint32_t
TriggerADCTaskInit(void)
{
    // Create the queues
    g_pADCQueue = xQueueCreate(ADC_QUEUE_SIZE, ADC_ITEM_SIZE);

    if (g_pADCQueue == NULL)
    {
        UARTprintf("Error: ADC Queue creation failed\n");
        return(1);
    }

    //Initialise the ADC Interrupt
    ADCIntInit();

    // Create the Trigger ADC task.
    if(xTaskCreate(TriggerADCTask, (const portCHAR *)"Trigger ADC", TRIGGER_ADC_TASK_STACK_SIZE, NULL,
                   tskIDLE_PRIORITY + PRIORITY_TRIGGER_ADC_TASK, NULL) != pdTRUE)
    {
        UARTprintf("Error: Trigger task creation failed\n"); // Error indication
        return(1);
    }

    // Success.
    return(0);
}
