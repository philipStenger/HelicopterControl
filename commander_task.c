//*****************************************************************************
// File:    commander_task.c
// Authors: Henry Mandeno and Zac Morrow
// Date:    07/08/2023
// Group:   19
// Brief:   Commander finite state machine task that updates system state.
//          Takes Commander queue (user button inputs) and Real Position as inputs,
//          updates the system state and Target Position based on the inputs.
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
#include "controller_task.h"
#include "commander_task.h"

//*****************************************************************************
//
// Definitions and Globals
//
//*****************************************************************************

// Parameters for Commander Task
#define COMMANDER_TASK_STACK_SIZE           128 // Stack size in words
#define COMMANDER_TASK_DELAY_MS             50

// Helicopter altitude parameters
#define MAXIMUM_ALTITUDE                    100
#define ALTITUDE_STEP                       10
#define TAKEOFF_ALTITUDE                    10

// Helicopter yaw parameters
#define MAXIMUM_YAW                         360
#define MINIMUM_YAW                         0
#define YAW_STEP                            15
#define TAKEOFF_YAW                         15

typedef enum {
    IDLE,
    TAKEOFF,
    FLYING,
    LANDING
} State;

typedef enum {
    SET,
    INCREMENT,
    DECREMENT
} Operation;

// Mutexes
xSemaphoreHandle g_pTargetPositionMutex;
extern xSemaphoreHandle g_pUARTMutex;
extern xSemaphoreHandle g_pRealAltitudeMutex;

// Queues
extern xQueueHandle g_pCommanderQueue;

// Global variables
static State state = IDLE;
static HelicopterPosition targetPosition;

//*****************************************************************************
//
// Functions
//
//*****************************************************************************

/**
 * @brief Updates the target position based on provided altitude, yaw, and operation.
 */
void
UpdateTargetPosition(uint32_t i32Altitude, uint16_t i16Yaw, uint8_t operation)
{
    xSemaphoreTake(g_pTargetPositionMutex, portMAX_DELAY);

    switch (operation)
    {
        case SET:
            targetPosition.altitude = i32Altitude;
            targetPosition.yaw = i16Yaw;
            break;
        case INCREMENT:
            targetPosition.altitude += i32Altitude;
            targetPosition.yaw += i16Yaw;
            break;
        case DECREMENT:
            targetPosition.altitude -= i32Altitude;
            targetPosition.yaw -= i16Yaw;
            break;
        default:
            break;
    }

    if (targetPosition.altitude > MAXIMUM_ALTITUDE) targetPosition.altitude = MAXIMUM_ALTITUDE;
    if (targetPosition.yaw > MAXIMUM_YAW) targetPosition.yaw = MAXIMUM_YAW;
    if (targetPosition.yaw < MINIMUM_YAW) targetPosition.yaw = MINIMUM_YAW;

    xSemaphoreGive(g_pTargetPositionMutex);
}

/**
 * @brief Retrieves the current target position.
 */
HelicopterPosition
GetTargetPosition(void)
{
    return targetPosition;
}

/**
 * @brief Commander task implementing a finite state machine for flight control.
 */
static void
CommanderTask(void *pvParameters)
{
    portTickType ui16LastTime;
    uint8_t i8FlightCommand;
    int32_t i32RealAltitude;

    while(1)
    {
        // Read the flight command, if available on queue.
        if(xQueueReceive(g_pCommanderQueue, &i8FlightCommand, 0) != pdPASS)
        {
            i8FlightCommand = NO_COMMAND;
        }

        // Read the current Real Altitude
        xSemaphoreTake(g_pRealAltitudeMutex, portMAX_DELAY);
        i32RealAltitude = GetRealAltitude();
        xSemaphoreGive(g_pRealAltitudeMutex);

        xSemaphoreTake(g_pUARTMutex, portMAX_DELAY);
        UARTprintf("Target Altitude: %d Current altitude: %d\n", targetPosition.altitude, i32RealAltitude);
        xSemaphoreGive(g_pUARTMutex);

        // Evaluate if a state change should be performed, and update control system parameters if entering a new state.
        switch(state) {
            case IDLE:
                // Enter TAKEOFF state if UP command received.
                if(i8FlightCommand == UP)
                {
                    UpdateTargetPosition(TAKEOFF_ALTITUDE, TAKEOFF_YAW, SET);
                    state = TAKEOFF;
                    xSemaphoreTake(g_pUARTMutex, portMAX_DELAY);
                    UARTprintf("ILDE: Entering TAKEOFF state\n");
                    xSemaphoreGive(g_pUARTMutex);
                }
                break;

            case TAKEOFF:
                // Once TAKEOFF_ALTITUDE altitude is reached, enter FLYING state.
                if (i32RealAltitude >= TAKEOFF_ALTITUDE)
                {
                    state = FLYING;
                    xSemaphoreTake(g_pUARTMutex, portMAX_DELAY);
                    UARTprintf("TAKEOFF: Entering FLYING state\n");
                    xSemaphoreGive(g_pUARTMutex);
                }
                break;

            case FLYING:
                // Update target position if a new command has been received.
                if(i8FlightCommand == UP)
                {
                    UpdateTargetPosition(ALTITUDE_STEP, 0, INCREMENT);
                }
                else if (i8FlightCommand == DOWN)
                {
                    UpdateTargetPosition(ALTITUDE_STEP, 0, DECREMENT);
                }
                else if (i8FlightCommand == LEFT)
                {
                    UpdateTargetPosition(0, YAW_STEP, INCREMENT);
                }
                else if (i8FlightCommand == RIGHT)
                {
                    UpdateTargetPosition(0, YAW_STEP, DECREMENT);
                }

                // Enter LANDING state if Target Altitude has been set to 0.
                if (targetPosition.altitude == 0)
                {
                    state = LANDING;
                    UpdateTargetPosition(0, 0, SET);
                    xSemaphoreTake(g_pUARTMutex, portMAX_DELAY);
                    UARTprintf("FLYING: Entering Landing state\n");
                    xSemaphoreGive(g_pUARTMutex);
                }
                break;

            case LANDING:
                // If Real Altitude = 0, enter IDLE state.
                if (i32RealAltitude == 0)
                {
                    state = IDLE;
                    xSemaphoreTake(g_pUARTMutex, portMAX_DELAY);
                    UARTprintf("LANDING: Entering IDLE state\n");
                    xSemaphoreGive(g_pUARTMutex);
                }
                break;
            default:
                // Should never enter here.
                state = IDLE;
                break;
        }

        vTaskDelayUntil(&ui16LastTime, COMMANDER_TASK_DELAY_MS / portTICK_RATE_MS);
    }
}

/**
 * @brief Initializes the Commander Task and associated resources.
 */
uint8_t
CommanderTaskInit(void)
{
    g_pTargetPositionMutex = xSemaphoreCreateMutex();

    if(xTaskCreate(CommanderTask, (const portCHAR *)"Commander", COMMANDER_TASK_STACK_SIZE, NULL,
                   tskIDLE_PRIORITY + PRIORITY_COMMANDER_TASK, NULL) != pdTRUE)
    {
        return(1);
    }

    return(0);
}
