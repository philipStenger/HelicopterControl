#ifndef __BUTTON_TASKS_H__
#define __BUTTON_TASKS_H__

//*****************************************************************************
// File:    button_tasks.h
// Authors: Henry Mandeno and Zac Morrow
// Date:    07/08/2023
// Group:   19
// Brief:   Button tasks that poll (debounce) buttons and check button state, and
//          send command to Commander queue if a button has been pressed.
//*****************************************************************************

/**
 * @brief Initialises both the Button Check Task and The Button Update Task.
 * 
 * @return uint8_t 0 on success, 1 on failure.
 */
extern uint8_t
ButtonTasksInit(void);

#endif // __BUTTON_TASKS_H__
