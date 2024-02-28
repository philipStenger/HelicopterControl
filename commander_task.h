#ifndef __COMMANDER_TASK_H__
#define __COMMANDER_TASK_H__

//*****************************************************************************
// File:    commander_task.h
// Authors: Henry Mandeno and Zac Morrow
// Date:    07/08/2023
// Group:   19
// Brief:   Commander finite state machine task that updates system state.
//          Takes Commander queue (user button inputs) and Real Position as 
//          inputs, updates the system state and Target Position based on the
//          inputs.
//*****************************************************************************

/**
 * @brief HelicopterPosition struct definition.
 */
typedef struct HelicopterPosition {
    int32_t  altitude;
    int32_t  yaw;
} HelicopterPosition;

/**
 * @brief  Get the Target Position object.
 * 
 * @return HelicopterPosition The current Target Position object.
 */
HelicopterPosition
GetTargetPosition(void);

/**
 * @brief  Initialises the Commander Task.
 * 
 * @return uint_8 0 on success, 1 on failure.
 */
extern uint8_t
CommanderTaskInit(void);

#endif // __COMMANDER_TASK_H__
