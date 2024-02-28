#ifndef CONTROLLER_TASK_H_
#define CONTROLLER_TASK_H_

//*****************************************************************************
// File:    controller_task.h
// Authors: Henry Mandeno and Zac Morrow
// Date:    07/08/2023
// Group:   19
// Brief:   Controller task that initializes and updates the main and tail
//          PID controllers, and send the PID control output to motor drivers.
//*****************************************************************************

/**
 * @brief Structure representing the error in position for altitude and yaw.
 */
typedef struct Error {
    float altitude;
    float yaw;
} Error;

/**
 * @brief   Returns the mostly recently saved real altitude value.
 *
 * @returns int32_t Current altitude of helicopter.
 */
int32_t
GetRealAltitude(void);

/**
 * @brief   Intialises the Controller Task.
 *
 * @returns uint32_t 0 on success, 1 on failure.
 */
extern uint8_t
ControllerTaskInit(void);

#endif /* CONTROLLER_TASK_H_ */
