//*****************************************************************************
// File:        encoder.c
// Authors:     Philip Stenger and Luke Peters
// Modified:    Henry Mandeno and Zac Morrow
// Date:        21/08/2023
// Group:       19
// Brief:       Module to process input from a Quadrature Encoder
//*****************************************************************************

#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <inc/hw_memmap.h>
#include "inc/hw_ints.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"
#include "utils/uartstdio.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "encoder.h"

//*****************************************************************************
//
// Definitions and Globals
//
//*****************************************************************************

#define TESTING                 1
#define COUNTS_PER_REVOLUTION   448

// The item and queue sizes
#define ENCODER_ITEM_SIZE       sizeof(uint8_t)
#define ENCODER_QUEUE_SIZE      5


// Static encoder variables
static uint8_t ui8PreviousEncoderState;
static int16_t i16EncoderCount = 0;

// Queues
xQueueHandle g_pEncoderQueue;

//*****************************************************************************
//
// Functions
//
//*****************************************************************************

#ifdef TESTING

/**
 * @brief Sets the previous state of the encoder.
 */
void 
SetPreviousState(uint8_t ui8TestingPreviousState)
{
    ui8PreviousEncoderState = ui8TestingPreviousState;
}

/**
 * @brief Sets the encoder count.
 */
void 
SetEncoderCount(int16_t i16TestingEncoderCount)
{
    i16EncoderCount = i16TestingEncoderCount;
}

/**
 * @brief Gets the previous state of the encoder.
 */
uint8_t 
GetPreviousState(void)
{
    return ui8PreviousEncoderState;
}

/**
 * @brief Gets the encoder count.
 */
int16_t 
GetEncoderCount(void)
{
    return i16EncoderCount;
}

#endif

/**
 * @brief Handles the quadrature encoder interrupts, reading the pin states
 *        and sending them to the Encoder Queue.
 */
void 
QuadEncoderISRHandler(void)
{
    uint8_t ui8EncoderState = ReadEncoderState();
    xQueueSendFromISR(g_pEncoderQueue, &ui8EncoderState, NULL);

    GPIOIntClear(GPIO_PORTB_BASE, GPIO_INT_PIN_0 | GPIO_INT_PIN_1);
}

/**
 * @brief Initialize pin-change interrupts for the quadrature encoder.
 */
void 
QuadEncoderIntInit(void)
{
    GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_PIN_0, GPIO_BOTH_EDGES);
    GPIOIntEnable(GPIO_PORTB_BASE, GPIO_PIN_0);

    GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_PIN_1, GPIO_BOTH_EDGES);
    GPIOIntEnable(GPIO_PORTB_BASE, GPIO_PIN_1);

    IntRegister(INT_GPIOB_TM4C123, &QuadEncoderISRHandler);
    IntPrioritySet(INT_GPIOB_TM4C123, pdTM4C_RTOS_INTERRUPT_PRIORITY(6));
    IntEnable(INT_GPIOB_TM4C123);
}

/**
 * @brief Reads the encoder state from the GPIO pins. The encoder state
 *        is a 2-bit number of the form AB, where A is the state of pin A
 *        and B is the state of pin B.
 */
uint8_t 
ReadEncoderState(void)
{
    uint8_t ui8pinAState;
    uint8_t ui8pinBState;
    uint8_t ui8encoderState;

    // Read pin states from GPIO
    ui8pinAState = (GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_0) ? 1 : 0);
    ui8pinBState = (GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_1) ? 1 : 0);

    // Combine pin states into encoder state    
    ui8encoderState = (ui8pinBState << 1) | ui8pinAState;

    return ui8encoderState;
}

/**
 * @brief Calculates the yaw from the encoder state by using the exclusive OR
 *        of the current and previous states to determine the direction of rotation.
 *        This direction is used to increment or decrement the encoder count.
 *        The encoder count is then transformed into yaw in degrees.
 */
uint16_t
CalculateYawFromEncoder(uint8_t ui8CurrentEncoderState)
{
    static uint16_t ui16Yaw = 0;
    
    // Read encoder state by extracting the phase states
    uint8_t ui8CurrentPhaseA = ui8CurrentEncoderState & 0x01; 
    uint8_t ui8CurrentPhaseB = (ui8CurrentEncoderState >> 1) & 0x01; 

    uint8_t ui8PreviousPhaseA = ui8PreviousEncoderState & 0x01;
    uint8_t ui8PreviousPhaseB = (ui8PreviousEncoderState >> 1) & 0x01;

    if (ui8PreviousPhaseA ^ ui8CurrentPhaseB)                                       
    {
        // Increment encoder count
        i16EncoderCount++;

        // Wrap between 0 and 448
        i16EncoderCount %= COUNTS_PER_REVOLUTION;                  

    } else if (ui8CurrentPhaseA ^ ui8PreviousPhaseB)
    {
        // Decrement encoder count
        i16EncoderCount--;

        // Wrap between 0 and 448
        if (i16EncoderCount == -1)
        {
            i16EncoderCount = COUNTS_PER_REVOLUTION;
        }
    }

    // Update previous state
    ui8PreviousEncoderState = ui8CurrentEncoderState;

    // Transforms encoder count into yaw in degrees
    ui16Yaw = (uint16_t)round(i16EncoderCount*360/COUNTS_PER_REVOLUTION);

    // Wrap between 0 and 359
    ui16Yaw %= 360;                                                                

    return ui16Yaw;
}

/**
 * @brief Initializes encoder peripherals.
 */
void
EncoderInit(void)
{
    // Configure GPIO peripherals
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB));

    // Set pins to inputs
    GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_0); 
    GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_1); 

    // Initialise interrupt for quadrature encoder
    QuadEncoderIntInit();

    // Create the encoder queue
    g_pEncoderQueue = xQueueCreate(ENCODER_QUEUE_SIZE, ENCODER_ITEM_SIZE);

    // Check for errors.
    if (g_pEncoderQueue == NULL)
    {
        UARTprintf("Error: Encoder queue creation failed\n");
    }
}


