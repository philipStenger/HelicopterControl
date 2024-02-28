#ifndef ALTITUDE_TASK_H_
#define ALTITUDE_TASK_H_

//*****************************************************************************
// File:    altitude_task.h
// Authors: Luke Peters and Philip Stenger
// Date:    07/08/2023
// Group:   19
// Brief:   Takes readings from the ADC queue and converts them to altitudes.
//*****************************************************************************

/**
 * @brief Intialises the Altitude Task.
 *
 * @returns uint32_t 0 on success, 1 on failure.
 */
extern uint8_t
AltitudeTaskInit(void);

#endif /* ALTITUDE_TASK_H_ */
