//*****************************************************************************
// File:    controller_task.c
// Authors: Henry Mandeno and Zac Morrow
// Date:    07/08/2023
// Group:   19
// Brief:   Controller task that initializes and updates the main and tail
//          PID controllers, and send the PID control output to motor drivers.
//*****************************************************************************

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "driverlib/pwm.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "drivers/pwm.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "priorities.h"
#include "pid.h"
#include "semphr.h"
#include "commander_task.h"
#include "yaw_reading_task.h"
#include "controller_task.h"

//*****************************************************************************
//
// Globals and Definitions
//
//*****************************************************************************

//      PID Initialisation Details
//---Main Rotor PID
#define MAIN_CONTROL_KP             2.3f
#define MAIN_CONTROL_KI             1.0f
#define MAIN_CONTROL_KD             0.5f
#define MAIN_CONTROL_ACCUM_LIMIT    150.0f
#define MAIN_SETTLE_DUTY_CYCLE      50

//---Tail Rotor PID
#define TAIL_CONTROL_KP             2.5f
#define TAIL_CONTROL_KI             0.05f
#define TAIL_CONTROL_KD             0.2f
#define TAIL_CONTROL_ACCUM_LIMIT    100.0f
#define TAIL_SETTLE_DUTY_CYCLE      50

#define DUTY_CYCLE_MAX              98.0f
#define DUTY_CYCLE_MIN              2.0f

// Parameters for Controller Task
#define CONTROLLER_TASK_STACK_SIZE  128         // Stack size in words
#define CONTROLLER_TASK_DELAY_MS    20    
#define CONTROLLER_TASK_DELAY_S     (float)CONTROLLER_TASK_DELAY_MS / 1000.0f

// Queue Parameters
#define CONTROLLER_ITEM_SIZE        sizeof(uint8_t)
#define CONTROLLER_QUEUE_SIZE       5

// Mutexes
xSemaphoreHandle g_pRealAltitudeMutex;
extern xSemaphoreHandle g_pYawMutex;
extern xSemaphoreHandle g_pTargetPositionMutex;

// Queues
extern xQueueHandle g_pAltitudeQueue;

// Structs
static HelicopterPosition realPosition;
static HelicopterPosition targetPositionCopy;

static PID mainPID;
static PID tailPID;

static PWM mainPWM;
static PWM tailPWM;

static Error positionError;

//*****************************************************************************
//
// Functions
//
//*****************************************************************************

/**
 * @brief Initialise PID instances for the Main and Tail Rotor.
 */
void
ControlInit(void)
{
    PIDInit(&mainPID, MAIN_CONTROL_KP, MAIN_CONTROL_KI, MAIN_CONTROL_KD, MAIN_CONTROL_ACCUM_LIMIT, MAIN_SETTLE_DUTY_CYCLE);
    PIDInit(&tailPID, TAIL_CONTROL_KP, TAIL_CONTROL_KI, TAIL_CONTROL_KD, TAIL_CONTROL_ACCUM_LIMIT, TAIL_SETTLE_DUTY_CYCLE);
}

/**
 * @brief Initialise PWM instances for the Main and Tail Rotor and sets duty cycle to zero.
 */
void
RotorsInit(void)
{
    PWMConfigInit(&mainPWM, PWM_MAIN_PERIPH_PWM, PWM_MAIN_GEN, PWM_MAIN_BASE, PWM_MAIN_OUTNUM, PWM_MAIN_OUTBIT, PWM_MAIN_PERIPH_GPIO,
            PWM_MAIN_GPIO_BASE, PWM_MAIN_GPIO_CONFIG, PWM_MAIN_GPIO_PIN, PWM_MAIN_NAME);
    PWMInit(&mainPWM);

    PWMConfigInit(&tailPWM, PWM_TAIL_PERIPH_PWM, PWM_TAIL_GEN, PWM_TAIL_BASE, PWM_TAIL_OUTNUM, PWM_TAIL_OUTBIT, PWM_TAIL_PERIPH_GPIO,
                PWM_TAIL_GPIO_BASE, PWM_TAIL_GPIO_CONFIG, PWM_TAIL_GPIO_PIN, PWM_TAIL_NAME);
    PWMInit(&tailPWM);

    PWMUpdateDutyCycle(&mainPWM, 0);
    PWMUpdateDutyCycle(&tailPWM, 0);
}

/**
 * @brief Ensures the duty cycle stays within peripheral limits.
 */
float
LimitDutyCycle(float fDutyCycle)
{
    if (fDutyCycle > DUTY_CYCLE_MAX)
    {
        fDutyCycle = DUTY_CYCLE_MAX;
    }
    else if (fDutyCycle < DUTY_CYCLE_MIN)
    {
        fDutyCycle = DUTY_CYCLE_MIN;
    }
    return fDutyCycle;
}

/**
 * @brief Updates the control loop with new error values and calculates motor control signals.
 *
 * This function takes an error value and a time step as inputs, updates the PID controllers
 * with the provided error values, calculates the control signals for the main and tail motors
 * using the PID controllers, limits the control values to predefined upper and lower limits,
 * and sends the resulting duty cycle signals to the motor drivers.
 */
void
ControlUpdate(Error positionError, float fDT)
{
    // Update the PID controllers with the new errors
    PIDUpdate(&mainPID, positionError.altitude, fDT);
    PIDUpdate(&tailPID, positionError.yaw, fDT);

    // Get new control values
    float fMainDutyCycle = PIDGetCommand(&mainPID);
    float fTailDutyCycle = PIDGetCommand(&tailPID);

    // Bound control values to upper/lower limits
    fMainDutyCycle = LimitDutyCycle(fMainDutyCycle);
    fTailDutyCycle = LimitDutyCycle(fTailDutyCycle);

    // Send outputs to motor drivers
    PWMUpdateDutyCycle(&mainPWM, fMainDutyCycle);
    PWMUpdateDutyCycle(&tailPWM, fTailDutyCycle);
}

/**
 * @brief Returns the real altitude value.
 */
int32_t
GetRealAltitude(void)
{
    return realPosition.altitude;
}

/**
 * @brief Updates position error using target and real positions.
 * 
 * This function updates altitude and yaw errors. It protects access to the target position 
 * via and yaw values using mutexes and retrieves real altitude from a queue. The function 
 * then calculates error and updates positionError values for altitude and yaw.
 */
void
UpdatePositionError(void)
{
    // Read the current Target Position from Flight Commander
    xSemaphoreTake(g_pTargetPositionMutex, portMAX_DELAY);
    targetPositionCopy = GetTargetPosition();
    xSemaphoreGive(g_pTargetPositionMutex);

    // Read the current Altitude Position, if available on queue
    xSemaphoreTake(g_pRealAltitudeMutex, portMAX_DELAY);
    xQueueReceive(g_pAltitudeQueue, &realPosition.altitude, 0);
    positionError.altitude = (float)(targetPositionCopy.altitude - realPosition.altitude);
    xSemaphoreGive(g_pRealAltitudeMutex);

    // Read the current Yaw Position
    xSemaphoreTake(g_pYawMutex, portMAX_DELAY);
    realPosition.yaw = GetYaw();
    xSemaphoreGive(g_pYawMutex);

    positionError.yaw = (float)(targetPositionCopy.yaw - realPosition.yaw);
}

/**
 * @brief Controller task that updates control loop and sends signals to motor drivers.
 *
 * This task continuously updates the position error, calculates control signals,
 * and sends the resulting signals to the motor drivers.
 */
static void
ControllerTask(void *pvParameters)
{
    portTickType ui16LastTime;

    ControlInit();
    RotorsInit();

    // Loop forever.
    while(1)
    {

        // Update Error
        UpdatePositionError();

        // Update Controller
        ControlUpdate(positionError, CONTROLLER_TASK_DELAY_S);
        

        vTaskDelayUntil(&ui16LastTime, CONTROLLER_TASK_DELAY_MS / portTICK_RATE_MS);
    }
}

/**
 * @brief Initializes the Controller task and associated resources.
 */
uint8_t
ControllerTaskInit(void)
{
    g_pRealAltitudeMutex = xSemaphoreCreateMutex();

    // Create the FSM task.
    if(xTaskCreate(ControllerTask, (const portCHAR *)"Controller", CONTROLLER_TASK_STACK_SIZE, NULL,
                   tskIDLE_PRIORITY + PRIORITY_CONTROLLER_TASK, NULL) != pdTRUE)
    {
        return(1);
    }

    // Success.
    return(0);
}
