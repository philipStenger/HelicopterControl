#ifndef BUTTONS_H_
#define BUTTONS_H_

//*****************************************************************************
// File:     buttons.h
// Author:   P.J. Bones UCECE
// Modified: Henry Mandeno and Zac Morrow
// Date:     08/08/2023
// Group:    19
// Brief:    Support for a set of buttons on the Tiva/Orbit platform.
//           The buttons are:  UP, DOWN, LEFT, RIGHT.
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>

//*****************************************************************************
// Constants
//*****************************************************************************
enum butNames {
    UP = 0, 
    DOWN, 
    LEFT, 
    RIGHT, 
    NUM_BUTS, 
    NO_COMMAND
};
enum butStates {
    RELEASED = 0, 
    PUSHED,
    NO_CHANGE
};

// Button hardware details
#define UP_BUT_PERIPH       SYSCTL_PERIPH_GPIOE
#define UP_BUT_PORT_BASE    GPIO_PORTE_BASE
#define UP_BUT_PIN          GPIO_PIN_0
#define UP_BUT_INACTIVE     false

#define DOWN_BUT_PERIPH     SYSCTL_PERIPH_GPIOD
#define DOWN_BUT_PORT_BASE  GPIO_PORTD_BASE
#define DOWN_BUT_PIN        GPIO_PIN_2
#define DOWN_BUT_INACTIVE   false

#define LEFT_BUT_PERIPH     SYSCTL_PERIPH_GPIOF
#define LEFT_BUT_PORT_BASE  GPIO_PORTF_BASE
#define LEFT_BUT_PIN        GPIO_PIN_4
#define LEFT_BUT_INACTIVE   true

#define RIGHT_BUT_PERIPH    SYSCTL_PERIPH_GPIOF
#define RIGHT_BUT_PORT_BASE GPIO_PORTF_BASE
#define RIGHT_BUT_PIN       GPIO_PIN_0
#define RIGHT_BUT_INACTIVE  true

// Debounce algorithm: A state machine is associated with each button.
// A state change occurs only after NUM_BUT_POLLS consecutive polls have
// read the pin in the opposite condition, before the state changes and
// a flag is set.  Set NUM_BUT_POLLS according to the polling rate.
#define NUM_BUT_POLLS 3

/**
 * @brief Initialise the variables associated with the set of buttons
 *        defined by the constants above.
 */
void
InitButtons(void);

/**
 * @brief Function designed to be called regularly. It polls all
 *        buttons once and updates variables associated with the buttons if
 *        necessary. It is efficient enough to be part of an ISR.
 */
void
UpdateButtons(void);

/**
 * @brief  Function returns the new button state if the button state
 *         (PUSHED or RELEASED) has changed since the last call, otherwise
 *         returns NO_CHANGE.
 * 
 * @param  butName Button to check. Must be one of butNums.
 * @return uint8_t Button state. Will be one of butStates.
 */
uint8_t
CheckButton(uint8_t butName);

#endif /*BUTTONS_H_*/
