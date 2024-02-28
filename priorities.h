#ifndef __PRIORITIES_H__
#define __PRIORITIES_H__

//*****************************************************************************
// File:    priorities.h
// Authors: Luke Peters, Philip Stenger, Henry Mandeno, and Zac Morrow
// Date:    07/08/2023
// Group:   19
// Brief:   Sets the priorites level for each of the tasks.
//          Low number -> low priority
//*****************************************************************************

#define PRIORITY_CONTROLLER_TASK        1
#define PRIORITY_BUTTON_UPDATE_TASK     2
#define PRIORITY_TRIGGER_ADC_TASK       3
#define PRIORITY_ALTITUDE_TASK          4
#define PRIORITY_YAW_READING_TASK       4
#define PRIORITY_BUTTON_CHECK_TASK      5
#define PRIORITY_COMMANDER_TASK         5

#endif // __PRIORITIES_H__
