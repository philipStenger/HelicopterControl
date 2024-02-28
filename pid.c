//*****************************************************************************
// File:     pid.c
// Author:   Dr Ben Mitchell
// Modified: Henry Mandeno and Zac Morrow
// Date:     07/08/2023
// Group:    Group 19
// Brief:    PID Controller module
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "pid.h"

//*****************************************************************************
//
// Functions
//
//*****************************************************************************

/**
 * @brief Initializes the PID controller parameters.
 */
void
PIDInit(PID* pid, float fProportionalGain, float fIntegralGain, float fDerivativeGain, float fAccumLimit, float fSettlePoint)
{
    pid->kp = fProportionalGain;
    pid->ki = fIntegralGain;
    pid->kd = fDerivativeGain;
    pid->accumulator = 0;
    pid->error = 0;
    pid->derivative = 0;
    pid->limit = fAccumLimit;
    pid->settlePoint = fSettlePoint;
}

/**
 * @brief Updates the PID controller with new error and time interval.
 */
void
PIDUpdate(PID* pid, float fError, float fDt)
{
    // Add the error to the integral
    pid->accumulator += fError * fDt;

    // Clamp the accumulator to the specified limit
    if (pid->accumulator > pid->limit) pid->accumulator = pid->limit;
    if (pid->accumulator < -pid->limit) pid->accumulator = -pid->limit;

    // Calculate the derivative
    pid->derivative = (fError - pid->error) / fDt;

    // Store the current error
    pid->error = fError;

}

/**
 * @brief Calculates and returns the control command based on PID calculations.
 */
float
PIDGetCommand(PID* pid)
{
    return pid->settlePoint + pid->kp * pid->error + pid->ki * pid->accumulator + pid->kd * pid->derivative;
}
