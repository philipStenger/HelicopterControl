//*****************************************************************************
// File:    altitude_task.c
// Authors: Luke Peters and Philip Stenger
// Date:    07/08/2023
// Group:   19
// Brief:   Takes readings from the ADC queue and converts them to altitudes.
//*****************************************************************************

#include <stdbool.h>
#include <stdint.h>
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "priorities.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "moving_average.h"

//*****************************************************************************
//
// Definitions and Globals
//
//*****************************************************************************

// Stack size in words
#define ALTITUDE_TASK_STACK_SIZE    128
#define ALTITUDE_TASK_DELAY_MS      20

// The item size and queue size for the Altitude queue.
#define ALTITUDE_ITEM_SIZE          sizeof(uint32_t)
#define ALTITUDE_QUEUE_SIZE         5

// Altitude Scaling Parameters
#define ALTITUDE_RANGE              1230
#define SAFETY_FACTOR               5
#define SCALE_FACTOR                100

// Queues
xQueueHandle g_pAltitudeQueue;
extern xQueueHandle g_pADCQueue;

// Mutexes
extern xSemaphoreHandle g_pUARTMutex;

// Global Variables
static MovingAverageBuffer altitudeBuffer;
static uint32_t ui32ZeroPoint = 0;

//*****************************************************************************
//
// Functions
//
//*****************************************************************************

/**
 * @brief Scales the averaged ADC value.
 */
uint32_t
ScaleAltitude(uint32_t ui32ADCAveraged)
{
    return (ui32ZeroPoint + SAFETY_FACTOR - ui32ADCAveraged) * SCALE_FACTOR/ ALTITUDE_RANGE;
}

/**
 * @brief Averages the ADC reading.
 */
uint32_t
AverageAltitude(uint32_t ui32ADCReading)
{
    return MovingAverageUpdate(&altitudeBuffer, ui32ADCReading);
}

/**
 * @brief Averages and scales the ADC reading.
 */
uint32_t
GetAltitude(uint32_t ui32ADCReading)
{
    uint32_t ui32ADCAveraged;

    ui32ADCAveraged = AverageAltitude(ui32ADCReading);

    return ScaleAltitude(ui32ADCAveraged);
}

/**
 * @brief Fills the buffer and sets the calibrated ADC value for 0 altitude.
 */
bool
CalibrateAltitude(uint32_t ui32ADCReading)
{
    bool isCalibrated = false;
    uint32_t ui32ADCAveraged = 0;
    static uint8_t ui8BufferSampleCount = 0;

    ui32ADCAveraged = MovingAverageUpdate(&altitudeBuffer, ui32ADCReading);
    ui8BufferSampleCount++;

    if (ui8BufferSampleCount >= MOVING_AVERAGE_BUFFER_SIZE)
    {
        isCalibrated = true;
        ui32ZeroPoint = ui32ADCAveraged;
    }
    return isCalibrated;
}

/**
 * @brief Initializes the moving average buffer.
 */
void
AltitudeBufferInit(void)
{
    MovingAverageInit(&altitudeBuffer);
}

/**
 * @brief Takes readings from the ADC queue and converts them to altitudes. 
 *        Then sends them to the Altitude Queue.
 */
static void
AltitudeTask(void *pvParameters)
{

    portTickType ui32WakeTime;
    uint32_t ui32ADCReading;
    static uint32_t ui32Altitude = 0;
    static bool isBufferFull = false;

    while(1)
    {
        if (xQueueReceive(g_pADCQueue, &ui32ADCReading, portMAX_DELAY) == pdTRUE)
        {
            // Ensures buffer is filled before readings are averaged.
            if (!isBufferFull)
            {
                isBufferFull = CalibrateAltitude(ui32ADCReading);
            }
            else
            {
                ui32Altitude = GetAltitude(ui32ADCReading);
            }
        }

        if (xQueueSend(g_pAltitudeQueue, &ui32Altitude, portMAX_DELAY) != pdPASS)
        {
            xSemaphoreTake(g_pUARTMutex, portMAX_DELAY);
            UARTprintf("Error: Altitude Queue is full\n");
            xSemaphoreGive(g_pUARTMutex);
            while(1)
            {
            }
        }

        // Wait for the required amount of time.
        vTaskDelayUntil(&ui32WakeTime, ALTITUDE_TASK_DELAY_MS / portTICK_RATE_MS);
    }
}

/**
 * @brief Intialises the Altitude task.
 */
uint8_t
AltitudeTaskInit(void)
{
    // Initialize the altitude buffer for averaging.
    AltitudeBufferInit();

    // Create the queue to send altitude values.
    g_pAltitudeQueue = xQueueCreate(ALTITUDE_QUEUE_SIZE, ALTITUDE_ITEM_SIZE);

    if (g_pAltitudeQueue == NULL)
    {
        UARTprintf("Error: Altitude Queue creation failed\n");
        return(1);
    }

    // Create the Altitude Task.
    if(xTaskCreate(AltitudeTask, (const portCHAR *)"Altitude", ALTITUDE_TASK_STACK_SIZE, NULL,
                   tskIDLE_PRIORITY + PRIORITY_ALTITUDE_TASK, NULL) != pdTRUE)
    {
        UARTprintf("Error: Altitude task creation failed\n");
        return(1);
    }

    // Success.
    return(0);
}
