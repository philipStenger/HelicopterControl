#ifndef MOVING_AVERAGE_H_
#define MOVING_AVERAGE_H_

//*****************************************************************************
// File:    moving_average.h
// Authors: Luke Peters and Philip Stenger
// Date:    07/08/2023
// Group:   19
// Brief:   Implements a moving average buffer.
//*****************************************************************************

// Buffer Size
#define MOVING_AVERAGE_BUFFER_SIZE 10

/**
 * @brief Moving average buffer structure.
 */
typedef struct {
    uint32_t buffer[MOVING_AVERAGE_BUFFER_SIZE];
    int index;
    uint32_t sum;
} MovingAverageBuffer;

/**
 * @brief Initializes a moving average buffer.
 *
 * @param buf: The pointer to the buffer's location.
 */
void
MovingAverageInit(MovingAverageBuffer* buf);

/**
 * @brief Add a new value to the buffer and return the new average.
 *
 * @param buf:       The pointer to the buffer's location.
 * @param newSample: The reading to be input to the buffer.
 *
 * @returns The average of the buffer.
 */
uint32_t
MovingAverageUpdate(MovingAverageBuffer* buf, uint32_t ui32NewSample);

#endif /* MOVING_AVERAGE_H_ */
