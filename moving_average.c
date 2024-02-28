//*****************************************************************************
// File:    moving_average.c
// Authors: Luke Peters and Philip Stenger
// Date:    07/08/2023
// Group:   19
// Brief:   Implements a moving average buffer.
//*****************************************************************************

#include <stdint.h>
#include <stdlib.h>
#include "moving_average.h"

//*****************************************************************************
//
// Functions
//
//*****************************************************************************

/**
 * @brief Initializes a moving average buffer.
 */
void
MovingAverageInit(MovingAverageBuffer* buf) {
    int i;

    buf->index = 0;
    buf->sum = 0;

    for (i = 0; i < MOVING_AVERAGE_BUFFER_SIZE; i++)
    {
        buf->buffer[i] = 0;
    }
}

/**
 * @brief Add a new value to the buffer and return the new average.
 */
uint32_t
MovingAverageUpdate(MovingAverageBuffer *buf, uint32_t ui32NewSample)
{
    uint32_t ui32MovingAverage;

    if (buf->index >= MOVING_AVERAGE_BUFFER_SIZE)
    {
        buf->index = 0; // Wrap around
    }

    buf->sum -= buf->buffer[buf->index];
    buf->sum += ui32NewSample;
    buf->buffer[buf->index] = ui32NewSample;

    buf->index++;

    ui32MovingAverage = buf->sum / MOVING_AVERAGE_BUFFER_SIZE;
    return ui32MovingAverage;
}
