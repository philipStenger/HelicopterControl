//*****************************************************************************
// File:    button_tasks.c
// Authors: Henry Mandeno and Zac Morrow
// Date:    07/08/2023
// Group:   19
// Brief:   Button tasks that poll (debounce) buttons, check button state, and
//          send command to Commander queue if a button has been pressed.
//*****************************************************************************

#include <stdbool.h>
#include <stdint.h>
#include "drivers/buttons.h"
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

// Parameters for Check Button State Task
#define BUTTONS_CHECK_STATE_TASK_STACK_SIZE     128  // Stack size in words
#define BUTTONS_CHECK_STATE_TASK_DELAY_MS       50

// Parameters for Poll Button Task
#define POLL_BUTTONS_TASK_STACK_SIZE            128  // Stack size in words
#define POLL_BUTTONS_TASK_DELAY_MS              2

// Commander Queue Parameters
#define COMMANDER_ITEM_SIZE                     sizeof(uint8_t)
#define COMMANDER_QUEUE_SIZE                    5

// Mutexes
extern xSemaphoreHandle g_pUARTMutex;

// Queues
xQueueHandle g_pCommanderQueue;

//*****************************************************************************
//
// Functions
//
//*****************************************************************************

/**
 * @brief Iterates over each button state and enqueues to commander queue if 
 *        button has been pressed.
 */
static void
CheckButtonStateTask(void *pvParameters)
{
    portTickType ui16LastTime;

    // Get the current tick count.
    ui16LastTime = xTaskGetTickCount();

    uint8_t ui8ControllerMessage;
    uint8_t ui8ButtonState = NO_CHANGE;
    uint8_t ui8ButtonBeingChecked = UP;

    while(1)
    {
        // Iterate over buttons and check each button state;
        // If a button has been pushed, enqueue to controller queue.
        for (ui8ButtonBeingChecked = UP; ui8ButtonBeingChecked < NUM_BUTS; ui8ButtonBeingChecked++)
        {
            ui8ButtonState = CheckButton(ui8ButtonBeingChecked);
            if (ui8ButtonState == PUSHED)
            {
                // Enqueue button command to Commander Queue.
                ui8ControllerMessage = ui8ButtonBeingChecked;
                if(xQueueSend(g_pCommanderQueue, &ui8ControllerMessage, portMAX_DELAY) !=
                               pdPASS)
                {
                    xSemaphoreTake(g_pUARTMutex, portMAX_DELAY);
                    UARTprintf("\nCommander Queue full. This should never happen.\n");
                    xSemaphoreGive(g_pUARTMutex);
                    while(1)
                    {
                    }
                }
            }
        }

        vTaskDelayUntil(&ui16LastTime, BUTTONS_CHECK_STATE_TASK_DELAY_MS / portTICK_RATE_MS);
    }
}

/**
 * @brief Button polling task. Performs button debouncing and updates individual
 *        button states. This task is designed to be called at a very high frequency
 *        as it polls the button GPIO pins.
 */
static void
PollButtonsTask(void *pvParameters)
{
    portTickType ui16LastTime;

    // Get the current tick count.
    ui16LastTime = xTaskGetTickCount();

    while(1)
    {
        UpdateButtons();
        vTaskDelayUntil(&ui16LastTime, POLL_BUTTONS_TASK_DELAY_MS / portTICK_RATE_MS);
    }
}

/**
 * @brief Initialises the Button Tasks and associated resources.
 */
uint8_t
ButtonTasksInit(void)
{
    // Initialise the buttons.
    InitButtons();

    // Initialise the commander queue.
    g_pCommanderQueue = xQueueCreate(COMMANDER_QUEUE_SIZE, COMMANDER_ITEM_SIZE);

    if (g_pCommanderQueue == NULL)
    {
        UARTprintf("Error: Commander Queue creation failed\n");
        return(1);
    }

    // Create the poll buttons task.
    if(xTaskCreate(PollButtonsTask, (const portCHAR *)"ButtonUpdate",
                   POLL_BUTTONS_TASK_STACK_SIZE, NULL, tskIDLE_PRIORITY +
                       PRIORITY_BUTTON_UPDATE_TASK, NULL) != pdTRUE)
    {
        return(1);
    }

    // Create the check button state task.
    if(xTaskCreate(CheckButtonStateTask, (const portCHAR *)"ButtonCheck",
                   BUTTONS_CHECK_STATE_TASK_STACK_SIZE, NULL, tskIDLE_PRIORITY +
                   PRIORITY_BUTTON_CHECK_TASK, NULL) != pdTRUE)
    {
        return(1);
    }

    return(0);
}

