#ifndef ENCODER_H_
#define ENCODER_H_

//*****************************************************************************
// File:        encoder.h
// Authors:     Philip Stenger and Luke Peters
// Modified by: Henry Mandeno and Zac Morrow
// Date:        21/08/2023
// Group:       Group 19
// Brief:       Module to process input from a Quadrature Encoder
//*****************************************************************************

/**
 * @brief Sets the previous state of the encoder.
 *
 * @param ui8TestingPreviousState The previous state of the encoder for black box test.
 */
void 
SetPreviousState(uint8_t ui8TestingPreviousState);

/**
 * @brief Sets the encoder count.
 *
 * @param i16TestingEncoderCount The encoder count for black box test.
 */
void 
SetEncoderCount(int16_t i16TestingEncoderCount);

/**
 * @brief Gets the previous state of the encoder.
 *
 * @return The previous state.
 */
uint8_t 
GetPreviousState(void);

/**
 * @brief Gets the encoder count.
 *
 * @return The encoder count.
 */
int16_t 
GetEncoderCount(void);

/**
 * @brief Reads the encoder state from the GPIO pins. The encoder state
 *        is a 2-bit number of the form AB, where A is the state of pin A
 *        and B is the state of pin B.
 *
 * @return The encoder state.
 */
uint8_t 
ReadEncoderState(void);

/**
 * @brief Calculates the yaw from the encoder state by using the exclusive OR
 *        of the current and previous states to determine the direction of rotation.
 *        This direction is used to increment or decrement the encoder count.
 *        The encoder count is then transformed into yaw in degrees.
 *
 * @return The yaw in degrees from 0 to 359.
 */
uint16_t 
CalculateYawFromEncoder(uint8_t ui8EncoderState);

/**
 * @brief Initializes encoder peripherals.
 */
void 
EncoderInit(void);

#endif /* ENCODER_H_ */
