#ifndef YAW_READING_TASK_H_
#define YAW_READING_TASK_H_

//*****************************************************************************
// File:    yaw_reading_task.h
// Authors: Philip Stenger and Luke Peters
// Date:    08/08/2023
// Group:   19
// Brief:   Task that reads from the encoder queue and converts the encoder
//          state to yaw.      
//*****************************************************************************

/**
 * @brief Gets the yaw.
 *
 * @return The yaw.
 */
uint16_t 
GetYaw(void);

/**
 * @brief Initializes the yaw reading task. Creates the encoder queue and yaw mutex. 
 *
 * @return 0 if successful, 1 if not.   
 */
extern uint8_t
YawReadingTaskInit(void);


#endif /* YAW_READING_TASK_H_ */
