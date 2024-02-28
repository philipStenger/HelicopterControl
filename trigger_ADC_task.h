#ifndef TRIGGER_ADC_TASK_H_
#define TRIGGER_ADC_TASK_H_

//*****************************************************************************
// File:    trigger_ADC_task.h
// Authors: Luke Peters and Philip Stenger
// Date:    07/08/2023
// Group:   19
// Brief:   The Trigger ADC Task triggers the ADC interrupt. The ISR then sends an ADC
//          sample over a queue to the Altitude Task.
//*****************************************************************************

/**
 * @brief Intialises the Trigger ADC Task.
 *
 * @returns uint32_t 0 on success, 1 on failure.
 */
extern uint32_t
TriggerADCTaskInit(void);

#endif /* TRIGGER_ADC_TASK_H_ */
