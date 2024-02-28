#ifndef PID_H
#define PID_H

//*****************************************************************************
// File:     pid.h
// Author:   Dr Ben Mitchell
// Modified: Henry Mandeno and Zac Morrow
// Date:     07/08/2023
// Group:    Group 19
// Brief:    PID Controller module
//*****************************************************************************

/**
 * @brief Structure representing a Proportional-Integral-Derivative (PID) controller.
 */
typedef struct PID {
    float accumulator;
    float limit;

    float error;
    float derivative;

    float kp, ki, kd;
    float settlePoint;
} PID;

/**
 * @brief Initialise a PID instance with appropriate gains.
 *
 * @param pid           Pointer to the PID structure
 * @param fKp           Proportional gain
 * @param fKi           Integral gain
 * @param fKd           Derivative gain
 * @param fAccumLimit   Maximum value the integral can reach (prevents wind-up)
 * @param fSettlePoint  Offset the returned PID control value (useful for duty cycle which cannot go negative)
 *
 */
void
PIDInit(PID* pid, float fKp, float fKi, float fKd, float fAccumLimit, float fSettlePoint);

/**
 * @brief Update a PID controller with new error
 *
 * @param pid    Pointer to the PID structure
 * @param fError Current error input
 * @param fDt    Change in time since the last update
 *
 */
void
PIDUpdate(PID* pid, float fError, float fDt);

/** @brief 
 * 
 * @param   pid   Pointer to the PID structure
 * @returns float Current output from the controller
*/ 
float PIDGetCommand(PID* pid);

#endif // PID_H
