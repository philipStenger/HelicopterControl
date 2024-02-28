//*****************************************************************************
// File:    black_box_test.c
// Authors: Philip Stenger and Luke Peters
// Date:    18/08/2023
// Group:   19
// Brief:   Black box test for the CalculateYawFromEncoder function.
//*****************************************************************************
#include <stdint.h>
#include <stdbool.h>
#include "encoder.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "black_box_test.h"

//*****************************************************************************
//
// Definitions
//
//*****************************************************************************

// Input values
#define TEST_INPUT_CURRENT_STATE    0b111
#define TEST_INPUT_PREV_STATE       0b10
#define TEST_INPUT_ENCODER_COUNT    428

// Output values
#define TEST_OUTPUT_YAW             344
#define TEST_OUTPUT_PREV_STATE      0b111
#define TEST_OUTPUT_ENCODER_COUNT   429

/**
 * @brief Black box test for the CalculateYawFromEncoder function.
 */
void 
BlackBoxTest(void)
{
    UARTprintf("--- Yaw Black Box Test Started ---\n");

    // Set the encoder count and previous state
    SetEncoderCount(TEST_INPUT_ENCODER_COUNT);
    SetPreviousState(TEST_INPUT_PREV_STATE);

    // Calculate the yaw from ISR
    uint16_t ui16OutputYaw = CalculateYawFromEncoder(TEST_INPUT_CURRENT_STATE);

    // Get the previous state and encoder count
    uint8_t ui8OutputPreviousState = GetPreviousState();
    int16_t i16OutputEncoderCount = GetEncoderCount();

    UARTprintf("Yaw: %u, Prev State: %u, Encoder Count: %d\n", ui16OutputYaw, ui8OutputPreviousState, i16OutputEncoderCount);

    // Check the yaw, previous state and encoder count are correct
    if (ui16OutputYaw != TEST_OUTPUT_YAW || ui8OutputPreviousState != TEST_OUTPUT_PREV_STATE || i16OutputEncoderCount != TEST_OUTPUT_ENCODER_COUNT)
    {
        UARTprintf("---Test Failed---\nReasons:\n");
    }
    else
    {
        UARTprintf("--- Test Passed ---\n");
    }

    if (ui16OutputYaw != TEST_OUTPUT_YAW)
    {
        UARTprintf("Yaw: %d | Expected yaw: %d\n", ui16OutputYaw, TEST_OUTPUT_YAW);
    }

    if (ui8OutputPreviousState != TEST_OUTPUT_PREV_STATE)
    {
        UARTprintf("Previous state: %d | Expected previous state: %d\n", ui8OutputPreviousState, TEST_OUTPUT_PREV_STATE);
    }

    if (i16OutputEncoderCount != TEST_OUTPUT_ENCODER_COUNT)
    {
        UARTprintf("Encoder count: %d | Expected encoder count: %d\n", i16OutputEncoderCount, TEST_OUTPUT_ENCODER_COUNT);
    }

    UARTprintf("Yaw Black Box Test Complete\n");
}
