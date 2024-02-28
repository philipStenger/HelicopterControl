//*****************************************************************************
// File:    yaw_reading_task.c
// Authors: Philip Stenger and Luke Peters
// Date:    08/08/2023
// Group:   19
// Brief:   Task that reads from the encoder queue and converts the encoder
//          state to yaw.      
//*****************************************************************************

#include <stdbool.h>
#include <stdint.h>
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "priorities.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "yaw_reading_task.h"
#include "encoder.h"

//*****************************************************************************
//
// Definitions and Globals
//
//*****************************************************************************

// Stack size in words
#define YAW_READING_TASK_STACK_SIZE     128

// Delay between altitude readings in milliseconds
#define YAW_READING_TASK_DELAY_MS       20

// The item and queue sizes
#define YAW_ITEM_SIZE                   sizeof(uint16_t)
#define YAW_QUEUE_SIZE                  1

// Static yaw value
static uint16_t ui16Yaw = 0;

// Queues
extern xQueueHandle g_pEncoderQueue;

// Mutexes
xSemaphoreHandle g_pYawMutex;

//*****************************************************************************
//
// Functions
//
//*****************************************************************************

/**
 * @brief Gets the yaw.
 */
uint16_t
GetYaw(void)
{
    return ui16Yaw;
}

/**
 * @brief Reads from the encoder queue and calculates the new yaw value based on the encoder state.
 */
static void
YawReadingTask(void *pvParameters)
{
    portTickType ui32WakeTime;
    uint8_t ui8EncoderState;

    while(1)
    {
        // Read from the encoder queue.
        if (xQueueReceive(g_pEncoderQueue, &ui8EncoderState, portMAX_DELAY) == pdTRUE)
        {
            // Calculate the yaw.
            xSemaphoreTake(g_pYawMutex, portMAX_DELAY);
            ui16Yaw = CalculateYawFromEncoder(ui8EncoderState);
            xSemaphoreGive(g_pYawMutex);
        }

        // Wait for the required amount of time.
        vTaskDelayUntil(&ui32WakeTime, YAW_READING_TASK_DELAY_MS / portTICK_RATE_MS);
    }
}

/**
 * @brief Initializes the yaw reading task. Creates the encoder queue and yaw mutex.       
 */
uint8_t
YawReadingTaskInit(void)
{
    // Initialize the encoder peripherals.
    EncoderInit();

    // Create the yaw mutex.
    g_pYawMutex = xSemaphoreCreateMutex();

    // Create the Yaw Reading Task.
    if(xTaskCreate(YawReadingTask, (const portCHAR *)"Encoder Reading", YAW_READING_TASK_STACK_SIZE, NULL,
                   tskIDLE_PRIORITY + PRIORITY_YAW_READING_TASK, NULL) != pdTRUE)
    {
        UARTprintf("Error: Encoder Reading task creation failed\n");
        return(1);
    }

    return(0);
}
