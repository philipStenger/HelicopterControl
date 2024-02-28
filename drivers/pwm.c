//*****************************************************************************
// File:    pwm.c
// Authors: Henry Mandeno and Zac Morrow
// Date:    07/08/2023
// Group:   19
// Brief:   Initialises and updates the PWM peripheral
//*****************************************************************************

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "drivers/buttons.h"
#include "drivers/pwm.h"
#include "utils/uartstdio.h"
#include "priorities.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

//*****************************************************************************
//
// Globals and Definitions
//
//*****************************************************************************

// PWM Peripheral Parameters
#define PWM_DUTY_INIT           50
#define PWM_RATE_HZ             200
#define PWM_DIVIDER             4
#define PWM_DIVIDER_CODE        SYSCTL_PWMDIV_4

//*****************************************************************************
//
// Functions
//
//*****************************************************************************

/**
 * @brief Calculate the period width for a given PWM frequency.
 */
uint32_t 
CalculatePeriodWidth(uint32_t ui32Frequency)
{
    return SysCtlClockGet() / PWM_DIVIDER / ui32Frequency;
}

/**
 * @brief Calculate the pulse width for a given duty cycle and PWM frequency.
 */
uint32_t 
CalculatePulseWidth(float fDutyCycle, uint32_t ui32Frequency)
{
    return CalculatePeriodWidth(ui32Frequency) * fDutyCycle / 100;
}

/**
 * @brief Initialize the PWM configuration.
 */
void 
PWMConfigInit(PWM* pwm, uint32_t pwmPeriph, uint32_t pwmGen, uint32_t pwmBase, uint32_t outNum, uint32_t outBit,
    uint32_t gpioPeriph, uint32_t gpioBase, uint32_t gpioConfig, uint32_t gpioPin, const char* pinName)
{
    // PWM configuration
    pwm->pwmPeriph = pwmPeriph;
    pwm->pwmGen = pwmGen;
    pwm->pwmBase = pwmBase;
    pwm->outNum = outNum;
    pwm->outBit = outBit;

    // GPIO configuration
    pwm->gpioPeriph = gpioPeriph;
    pwm->gpioBase = gpioBase;
    pwm->gpioConfig = gpioConfig;
    pwm->gpioPin = gpioPin;

    // Pin name
    pwm->pinName = pinName;

    // PWM characteristics
    pwm->duty = PWM_DUTY_INIT;
    pwm->frequency = PWM_RATE_HZ;
}

/**
 * @brief Initialize and configure the PWM peripheral.
 */
uint32_t 
PWMInit(PWM* pwm)
{
    // Set PWM clock divider
    SysCtlPWMClockSet(PWM_DIVIDER_CODE);

    // Initialize the GPIO and PWM peripherals
    SysCtlPeripheralEnable(pwm->pwmPeriph);
    SysCtlPeripheralEnable(pwm->gpioPeriph);

    while (!SysCtlPeripheralReady(pwm->pwmPeriph))
    {
    }

    GPIOPinConfigure(pwm->gpioConfig);
    GPIOPinTypePWM(pwm->gpioBase, pwm->gpioPin);

    PWMGenConfigure(pwm->pwmBase, pwm->pwmGen, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);

    // Set the initial PWM configuration
    PWMGenPeriodSet(pwm->pwmBase, pwm->pwmGen, CalculatePeriodWidth(pwm->frequency));
    PWMPulseWidthSet(pwm->pwmBase, pwm->outNum, CalculatePulseWidth(pwm->duty, pwm->frequency));

    // Enable PWM generation
    PWMGenEnable(pwm->pwmBase, pwm->pwmGen);
    PWMOutputState(pwm->pwmBase, pwm->outBit, true);

    return (0);
}

/**
 * @brief Update the PWM duty cycle.
 */
void 
PWMUpdateDutyCycle(PWM* pwm, float fDutyCycle)
{
    PWMPulseWidthSet(pwm->pwmBase, pwm->outNum, CalculatePulseWidth(fDutyCycle, pwm->frequency));
}
