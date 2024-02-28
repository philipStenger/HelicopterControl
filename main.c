//*****************************************************************************
// File:    main.c
// Authors: Henry Mandeno, Zac Morrow, Luke Peters, and Philip Stenger
// Date:    21/08/2023
// Group:   19
// Brief:   File where UART and tasks are configured and Task scheduler is started
//*****************************************************************************

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"
#include "utils/uartstdio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "trigger_ADC_task.h"
#include "altitude_task.h"
#include "yaw_reading_task.h"
#include "button_tasks.h"
#include "commander_task.h"
#include "controller_task.h"

//*****************************************************************************
//
// Globals and Definitions
//
//*****************************************************************************


//#define TESTING 1 // Uncomment to run Black Box test build

#ifdef TESTING
#include "black_box_test.h"
#endif

// The mutex that protects concurrent access of UART from multiple tasks.
xSemaphoreHandle g_pUARTMutex;

// The error routine that is called if the driver library encounters an error.
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

//*****************************************************************************
//
// Functions
//
//*****************************************************************************

/**
 * @brief This hook is called by FreeRTOS when an stack overflow error is detected.
 */
void
vApplicationStackOverflowHook(xTaskHandle *pxTask, char *pcTaskName)
{
    // This function can not return, so loop forever.  Interrupts are disabled
    // on entry to this function, so no processor interrupts will interrupt
    // this loop.
    while(1)
    {
    }
}


/**
 * @brief // Configure the UART and its pins.  This must be called before UARTprintf().
 */
void
ConfigureUART(void)
{
    // Enable the GPIO Peripheral used by the UART.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    // Enable UART0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    // Configure GPIO Pins for UART mode.
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // Use the internal 16MHz oscillator as the UART clock source.
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    // Initialize the UART for console I/O.
    UARTStdioConfig(0, 115200, 16000000);
}

/**
 * @brief Initialize FreeRTOS and start the initial set of tasks.
 */
int
main(void)
{

    // Set the clocking to run at 50 MHz from the PLL.
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
                       SYSCTL_OSC_MAIN);

    // Initialize the UART and configure it for 115,200, 8-N-1 operation.
    ConfigureUART();

    //Carry out black box testing if enabled
    #ifdef TESTING
    BlackBoxTest();
    while(1)
    {

    }
    #endif

    // Print introduction.
    UARTprintf("\n\nWelcome to the ENCE464 Helicopter Project\n");

    // Create a mutex to guard the UART.
    g_pUARTMutex = xSemaphoreCreateMutex();

    // Create the button tasks.
    if(ButtonTasksInit() != 0)
    {
        while(1)
        {
        }
    }

    // Create the ADC Averaging task.
    if(AltitudeTaskInit() != 0)
    {
        while(1)
        {
        }
    }

    // Create the Encoder Reading task.
    if(YawReadingTaskInit() != 0)
    {
        while(1)
        {
        }
    }

    // Create the trigger task.
    if(TriggerADCTaskInit() != 0)
    {
        while(1)
        {
        }
    }

    // Create the controller task
    if(CommanderTaskInit() != 0)
    {
        while(1)
        {
        }
    }

    // Create the controller task
    if(ControllerTaskInit() != 0)
    {
        while(1)
        {
        }
    }


    IntMasterEnable();
    UARTprintf("Interrupts enabled\n");

    // Start the scheduler. This should not return.
    UARTprintf("Starting Task Scheduler\n");
    vTaskStartScheduler();

    // In case the scheduler returns for some reason, print an error and loop forever.
    while(1)
    {
        UARTprintf("\n\nError: Exited scheduler - please restart\n");
    }
}


/**
 * @brief This is an error handling function called when FreeRTOS asserts.
 *        This should be used for debugging purposes.
 */
void vAssertCalled( const char * pcFile, unsigned long ulLine ) {
    (void)pcFile; // unused
    (void)ulLine; // unused
    while (1);
}
