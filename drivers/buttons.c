//*****************************************************************************
// File:     buttons.c
// Author:   P.J. Bones UCECE
// Modified: Henry Mandeno and Zac Morrow
// Date:     08/08/2023
// Group:    19
// Brief:    Support for a set of buttons on the Tiva/Orbit target platform.
//           The buttons are: UP, DOWN, LEFT, RIGHT.
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/tm4c123gh6pm.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/debug.h"
#include "buttons.h"

//*****************************************************************************
//
// Globals and Definitions
//
//*****************************************************************************
static bool buttonState[NUM_BUTS];    // Corresponds to the electrical state
static uint8_t buttonDebouncingCount[NUM_BUTS];
static bool hasButtonChanged[NUM_BUTS];
static bool buttonInactiveState[NUM_BUTS];   // Corresponds to the electrical state

//*****************************************************************************
//
// Functions
//
//*****************************************************************************

/**
 * @brief Initialise the variables associated with the set of buttons
 *        defined by the constants above.
 */
void
InitButtons (void)
{
    int i;

    // UP button (active HIGH)
    SysCtlPeripheralEnable(UP_BUT_PERIPH);
    GPIOPinTypeGPIOInput(UP_BUT_PORT_BASE, UP_BUT_PIN);
    GPIOPadConfigSet(UP_BUT_PORT_BASE, UP_BUT_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPD);
    buttonInactiveState[UP] = UP_BUT_INACTIVE;

    // DOWN button (active HIGH)
    SysCtlPeripheralEnable(DOWN_BUT_PERIPH);
    GPIOPinTypeGPIOInput(DOWN_BUT_PORT_BASE, DOWN_BUT_PIN);
    GPIOPadConfigSet(DOWN_BUT_PORT_BASE, DOWN_BUT_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPD);
    buttonInactiveState[DOWN] = DOWN_BUT_INACTIVE;

    // LEFT button (active LOW)
    SysCtlPeripheralEnable(LEFT_BUT_PERIPH);
    GPIOPinTypeGPIOInput(LEFT_BUT_PORT_BASE, LEFT_BUT_PIN);
    GPIOPadConfigSet(LEFT_BUT_PORT_BASE, LEFT_BUT_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
    buttonInactiveState[LEFT] = LEFT_BUT_INACTIVE;

    // RIGHT button (active LOW)
    // PF0 must be "unlocked" before reconfigured as GPIO Input
    SysCtlPeripheralEnable(RIGHT_BUT_PERIPH);
    GPIO_PORTF_LOCK_R = GPIO_LOCK_KEY;
    GPIO_PORTF_CR_R |= GPIO_PIN_0; //PF0 unlocked
    GPIO_PORTF_LOCK_R = GPIO_LOCK_M;
    GPIOPinTypeGPIOInput(RIGHT_BUT_PORT_BASE, RIGHT_BUT_PIN);
    GPIOPadConfigSet(RIGHT_BUT_PORT_BASE, RIGHT_BUT_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
    buttonInactiveState[RIGHT] = RIGHT_BUT_INACTIVE;

    for (i = 0; i < NUM_BUTS; i++)
    {
        buttonState[i] = buttonInactiveState[i];
        buttonDebouncingCount[i] = 0;
        hasButtonChanged[i] = false;
    }
}

/**
 * @brief Function designed to be called regularly. It polls all
 *        buttons once and updates variables associated with the buttons if
 *        necessary. It is efficient enough to be part of an ISR.
 */
void
UpdateButtons (void)
{
    bool buttonValue[NUM_BUTS];
    int i;

    // Read the pins; true means HIGH, false means LOW
    buttonValue[UP] = (GPIOPinRead(UP_BUT_PORT_BASE, UP_BUT_PIN) == UP_BUT_PIN);
    buttonValue[DOWN] = (GPIOPinRead(DOWN_BUT_PORT_BASE, DOWN_BUT_PIN) == DOWN_BUT_PIN);
    buttonValue[LEFT] = (GPIOPinRead(LEFT_BUT_PORT_BASE, LEFT_BUT_PIN) == LEFT_BUT_PIN);
    buttonValue[RIGHT] = (GPIOPinRead(RIGHT_BUT_PORT_BASE, RIGHT_BUT_PIN) == RIGHT_BUT_PIN);

    // Iterate through the buttons, updating button variables as required
    for (i = 0; i < NUM_BUTS; i++)
    {
        if (buttonValue[i] != buttonState[i])
        {
            buttonDebouncingCount[i]++;
            if (buttonDebouncingCount[i] >= NUM_BUT_POLLS)
            {
                buttonState[i] = buttonValue[i];
                hasButtonChanged[i] = true;    // Reset by call to checkButton()
                buttonDebouncingCount[i] = 0;
            }
        }
        else
        {
            buttonDebouncingCount[i] = 0;
        }
    }
}

/**
 * @brief Function returns the new button state if the button state
 *        (PUSHED or RELEASED) has changed since the last call, otherwise
 *        returns NO_CHANGE.
 */
uint8_t
CheckButton (uint8_t butName)
{
    if (hasButtonChanged[butName])
    {
        hasButtonChanged[butName] = false;
        if (buttonState[butName] == buttonInactiveState[butName])
            return RELEASED;
        else
            return PUSHED;
    }
    return NO_CHANGE;
}

