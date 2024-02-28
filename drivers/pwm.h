#ifndef __PWM_H__
#define __PWM_H__

//*****************************************************************************
// File:    pwm.h
// Authors: Henry Mandeno and Zac Morrow
// Date:    07/08/2023
// Group:   19
// Brief:   Initialises and updates the PWM peripheral.
//*****************************************************************************

// PWM Hardware Details
// Main Rotor PWM: M0PWM7, PC5, J4-05
#define PWM_MAIN_BASE           PWM0_BASE
#define PWM_MAIN_GEN            PWM_GEN_3
#define PWM_MAIN_OUTNUM         PWM_OUT_7
#define PWM_MAIN_OUTBIT         PWM_OUT_7_BIT
#define PWM_MAIN_PERIPH_PWM     SYSCTL_PERIPH_PWM0
#define PWM_MAIN_PERIPH_GPIO    SYSCTL_PERIPH_GPIOC
#define PWM_MAIN_GPIO_BASE      GPIO_PORTC_BASE
#define PWM_MAIN_GPIO_CONFIG    GPIO_PC5_M0PWM7
#define PWM_MAIN_GPIO_PIN       GPIO_PIN_5
#define PWM_MAIN_NAME           "MAIN ROTOR"

// Tail Rotor PWM: M1PWM5, PF1, J4-05
#define PWM_TAIL_BASE           PWM1_BASE
#define PWM_TAIL_GEN            PWM_GEN_2
#define PWM_TAIL_OUTNUM         PWM_OUT_5
#define PWM_TAIL_OUTBIT         PWM_OUT_5_BIT
#define PWM_TAIL_PERIPH_PWM     SYSCTL_PERIPH_PWM1
#define PWM_TAIL_PERIPH_GPIO    SYSCTL_PERIPH_GPIOF
#define PWM_TAIL_GPIO_BASE      GPIO_PORTF_BASE
#define PWM_TAIL_GPIO_CONFIG    GPIO_PF1_M1PWM5
#define PWM_TAIL_GPIO_PIN       GPIO_PIN_1
#define PWM_TAIL_NAME           "TAIL ROTOR"

/**
 * @brief Structure representing PWM configuration and characteristics.
 */
typedef struct PWM {
    uint32_t pwmPeriph;
    uint32_t pwmGen;
    uint32_t pwmBase;
    uint32_t outNum;
    uint32_t outBit;

    uint32_t gpioPeriph;
    uint32_t gpioBase;
    uint32_t gpioConfig;
    uint32_t gpioPin;

    const char *pinName;

    uint8_t duty;
    uint32_t frequency;
} PWM;

/**
 * @brief Initialize the PWM configuration.
 * 
 * @param pwm        Pointer to the PWM structure.
 * @param pwmPeriph  PWM peripheral to be used.
 * @param pwmGen     PWM generator.
 * @param pwmBase    PWM base address.
 * @param outNum     PWM output number.
 * @param outBit     PWM output bit.
 * @param gpioPeriph GPIO peripheral to be used.
 * @param gpioBase   GPIO base address.
 * @param gpioConfig GPIO configuration.
 * @param gpioPin    GPIO pin to be used for PWM output.
 * @param pinName    Name of the PWM pin.
 */
void
PWMConfigInit(PWM* pwm, uint32_t pwmPeriph, uint32_t pwmGen, uint32_t pwmBase, uint32_t outNum, uint32_t outBit,
    uint32_t gpioPeriph, uint32_t gpioBase, uint32_t gpioConfig, uint32_t gpioPin, const char* pinName);

/**
 * @brief Initialize and configure the PWM peripheral.
 * 
 * @param   pwm      Pointer to the PWM structure.
 * @returns uint32_t Returns 0 upon successful initialization.
 */
uint32_t PWMInit(PWM* pwm);

/**
 * @brief Update the PWM duty cycle.
 * 
 * @param pwm        Pointer to the PWM structure.
 * @param fDutyCycle New duty cycle as a percentage.
 */
void PWMUpdateDutyCycle(PWM* pwm, float fDutyCycle);

#endif // __PWM_H__
