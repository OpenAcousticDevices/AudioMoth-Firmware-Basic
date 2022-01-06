/****************************************************************************
 * gps.c
 * openacousticdevices.info
 * July 2021
 *****************************************************************************/

#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#include "em_cmu.h"
#include "em_emu.h"
#include "em_wdog.h"
#include "em_gpio.h"
#include "em_usart.h"
#include "em_timer.h"

#include "gps.h"
#include "nmeaparser.h"
#include "gpsutilities.h"

/* Hardware pinout constants */

#define MAGNETIC_SWITCH_GPIOPORT            gpioPortB
#define MAGNETIC_SWITCH_PIN                 9

#define GPS_ENABLE_N_GPIOPORT               gpioPortA
#define GPS_ENABLE_N_PIN                    7

#define UART_RX_GPIOPORT                    gpioPortB
#define UART_RX_PIN                         10

#define PPS_GPIOPORT                        gpioPortA
#define PPS_PIN                             8

/* GPS fix constants */

#define MINIMUM_VALID_PPS_TO_SET_TIME       4

#define MINIMUM_VALID_RMC_TO_SET_TIME       4

/* Useful time constants */

#define MILLISECONDS_IN_SECOND              1000

/* Timer constants */

#define TIMER_OVERFLOW_INTERVAL             (1 << 16)

/* RMC message parser results */

static GPS_fixTime_t fixTime;

static GPS_fixPosition_t fixPosition;

static NMEA_parserResultRMC_t parserResultRMC;

static NMEA_parserResultDEFAULT_t parserResultDEFAULT;

/* Volatile flags */

static volatile bool tick;

static volatile bool receivedPPS;

static volatile bool receivedRMC;

static volatile bool receivedDEFAULT;

static volatile bool cancelledBySwitch;

static volatile bool cancelledByMagneticSwitch;

static volatile uint32_t currentTimerCount;

static volatile uint32_t previousTimerCount;

/* Timer variables */

static uint32_t clockFrequency;

static uint32_t currentTickCounter;

static uint32_t maximumTickCounter;

/* Current PPS and RMC variables */

static volatile uint32_t currentPPSTime;

static volatile uint32_t currentPPSMilliSeconds;

static volatile uint32_t currentRMCTime;

static volatile uint32_t currentRMCMilliSeconds;

static volatile uint32_t currentMessageTime;

static volatile uint32_t currentMessageMilliSeconds;

/* Interrupt handler for RX */

void UART1_RX_IRQHandler() {

    /* Get the interrupt mask */

    uint32_t interruptMask = USART_IntGet(UART1);

    /* Handle the interrupt */

    if ((interruptMask & UART_IF_RXDATAV) && (UART1->STATUS & UART_STATUS_RXDATAV)) {

        /* Read the received byte */

        uint8_t byte = USART_Rx(UART1);

        /* Process using default parser */

        NMEA_parserStatus_t parserStatusDEFAULT = NMEAParser_parseDEFAULT(byte, &parserResultDEFAULT);

        if (parserStatusDEFAULT == NMEA_SUCCESS) {
            
            GPS_handleGetTime((uint32_t*)&currentMessageTime, (uint32_t*)&currentMessageMilliSeconds);

            receivedDEFAULT = true;

        }        

        /* Process using RMC parser */

        NMEA_parserStatus_t parserStatusRMC = NMEAParser_parseRMC(byte, &parserResultRMC);

        if (parserStatusRMC == NMEA_SUCCESS) {
            
            GPS_handleGetTime((uint32_t*)&currentRMCTime, (uint32_t*)&currentRMCMilliSeconds);

            receivedRMC = true;

        }

    }

    /* Clear the GPIO interrupt flag */

    USART_IntClear(UART1, interruptMask);

}

/* Interrupt handler for magnetic switch */

void GPIO_ODD_IRQHandler(void) {

    /* Get the interrupt mask */

    uint32_t interruptMask = GPIO_IntGet();

    /* Call the interrupt handler */

    if (interruptMask & (1 << MAGNETIC_SWITCH_PIN)) GPS_handleMagneticSwitchInterrupt();

    /* Clear the GPIO interrupt flag */

    GPIO_IntClear(interruptMask);

}

/* Interrupt handler for PPS */

void TIMER2_IRQHandler(void) {

    /* Get the interrupt mask */

    uint32_t interruptMask = TIMER_IntGet(TIMER2);

    /* Handle the interrupt */

    if (interruptMask & TIMER_IF_CC0) {
        
        GPS_handleGetTime((uint32_t*)&currentPPSTime, (uint32_t*)&currentPPSMilliSeconds);

        previousTimerCount = currentTimerCount;

        currentTimerCount = TIMER_CaptureGet(TIMER2, 0);

        receivedPPS = true;

    }

    if (interruptMask & TIMER_IF_OF) {

        currentTickCounter += 1;

        if (currentTickCounter > maximumTickCounter) {

            currentTickCounter = 0;

            tick = true;

        }

        WDOG_Feed();

    }

    /* Clear the interrupt flag */

    TIMER_IntClear(TIMER2, interruptMask);

}

/* Public functions */

void GPS_powerUpGPS() {

    GPIO_PinModeSet(GPS_ENABLE_N_GPIOPORT, GPS_ENABLE_N_PIN, gpioModePushPull, 0);

}

void GPS_powerDownGPS() {

    GPIO_PinModeSet(GPS_ENABLE_N_GPIOPORT, GPS_ENABLE_N_PIN, gpioModeDisabled, 0);

}

void GPS_enableGPSInterface() {

    /* Enable the RX pin */

    GPIO_PinModeSet(UART_RX_GPIOPORT, UART_RX_PIN, gpioModeInputPull, 1);

    /* Enable UART clock */

    CMU_ClockEnable(cmuClock_UART1, true);

    /* Initialise interface */

    USART_InitAsync_TypeDef uartInit = USART_INITASYNC_DEFAULT;

    uartInit.enable = usartDisable;

    uartInit.baudrate = 9600;

    USART_InitAsync(UART1, &uartInit);

    /* Prepare interrupts */

    USART_IntEnable(UART1, UART_IF_RXDATAV);

    NVIC_ClearPendingIRQ(UART1_RX_IRQn);

    NVIC_EnableIRQ(UART1_RX_IRQn);

    /* Enable route */

    UART1->ROUTE = UART_ROUTE_RXPEN | UART_ROUTE_LOCATION_LOC2;

    /* Enable UART */

    USART_Enable(UART1, usartEnableRx);

    /* Set up PPS pin and pull low */

    GPIO_PinModeSet(PPS_GPIOPORT, PPS_PIN, gpioModeInputPull, 0);

    /* Enable the ADC timer */

    CMU_ClockEnable(cmuClock_TIMER2, true);

    /* Initialise the ADC timer */

    TIMER_Init_TypeDef timerInit = TIMER_INIT_DEFAULT;

    timerInit.enable = false;

    TIMER_Init(TIMER2, &timerInit);

    /* Set up tick overflow */

    currentTickCounter = 0;

    clockFrequency = CMU_ClockFreqGet(cmuClock_TIMER2);

    maximumTickCounter = clockFrequency / TIMER_OVERFLOW_INTERVAL / GPS_TICK_EVENTS_PER_SECOND;

    TIMER_TopSet(TIMER2, TIMER_OVERFLOW_INTERVAL - 1);

    /* Initialise the ADC timer capture interrupt */

    TIMER_InitCC_TypeDef timerCCInit = {
        .eventCtrl  = timerEventEveryEdge,
        .edge       = timerEdgeRising,
        .prsSel     = timerPRSSELCh0,
        .cufoa      = timerOutputActionNone,
        .cofoa      = timerOutputActionNone,
        .cmoa       = timerOutputActionNone,
        .mode       = timerCCModeCapture,
        .filter     = false,
        .prsInput   = false,
        .coist      = false,
        .outInvert  = false,
    };

    TIMER_InitCC(TIMER2, 0, &timerCCInit);

    /* Initialise the interrupts */

    TIMER_IntEnable(TIMER2, TIMER_IF_CC0);

    TIMER_IntEnable(TIMER2, TIMER_IF_OF);

    NVIC_ClearPendingIRQ(TIMER2_IRQn);

    NVIC_EnableIRQ(TIMER2_IRQn);

    /* Start the timer */

    TIMER_Enable(TIMER2, true);

}

void GPS_disableGPSInterface(void) {

    /* Disable the RX UART */

    NVIC_DisableIRQ(UART1_RX_IRQn);

    USART_Reset(UART1);

    CMU_ClockEnable(cmuClock_UART1, false);

    /* Disable the PPS timer and interrupt */

    NVIC_DisableIRQ(TIMER2_IRQn);

    TIMER_Reset(TIMER2);

    CMU_ClockEnable(cmuClock_TIMER2, false);

    /* Disable the pins */

    GPIO_PinModeSet(UART_RX_GPIOPORT, UART_RX_PIN, gpioModeDisabled, 0);

    GPIO_PinModeSet(PPS_GPIOPORT, PPS_PIN, gpioModeDisabled, 0);

}

void GPS_enableMagneticSwitch() { 

    /* Enable the pin */
        
    GPIO_PinModeSet(MAGNETIC_SWITCH_GPIOPORT, MAGNETIC_SWITCH_PIN, gpioModeInputPull, 1);

    /* Enable interrupt on falling edge */

    GPIO_IntConfig(MAGNETIC_SWITCH_GPIOPORT, MAGNETIC_SWITCH_PIN, false, true, true);

    NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);

    NVIC_EnableIRQ(GPIO_ODD_IRQn);

}

void GPS_disableMagneticSwitch() { 

    /* Disable the pin */

    GPIO_PinModeSet(MAGNETIC_SWITCH_GPIOPORT, MAGNETIC_SWITCH_PIN, gpioModeDisabled, 0);

    /* Disable the interrupt */

    GPIO_IntConfig(MAGNETIC_SWITCH_GPIOPORT, MAGNETIC_SWITCH_PIN, false, false, false);

    NVIC_DisableIRQ(GPIO_ODD_IRQn);   

}

bool GPS_isMagneticSwitchClosed() {

    return GPIO_PinInGet(MAGNETIC_SWITCH_GPIOPORT, MAGNETIC_SWITCH_PIN) == 0;

}

GPS_fixResult_t GPS_setTimeFromGPS(uint32_t timeout) {

    uint32_t validRMC = 0;

    uint32_t validPPS = 0;

    uint32_t currentTime = 0;

    uint32_t timeToBeSetOnNextPPS = 0;

    /* Reset flag */

    receivedRMC = false;

    receivedPPS = false;

    cancelledBySwitch = false;

    cancelledByMagneticSwitch = false;

    /* Main loop */

    while (!cancelledBySwitch && !cancelledByMagneticSwitch && currentTime < timeout) {

        if (tick) {

            GPS_handleTickEvent();

            tick = false;

        }

        if (receivedDEFAULT && !(receivedRMC && parserResultRMC.status == 'A')) {

            GPS_handleMessageEvent(currentMessageTime, currentMessageMilliSeconds, parserResultDEFAULT.message);

            receivedDEFAULT = false;

        }

        if (receivedRMC) {

            if (parserResultRMC.status == 'A') {

                /* Call the handler */

                memcpy(&fixTime, &parserResultRMC, sizeof(GPS_fixTime_t));

                memcpy(&fixPosition, &parserResultRMC.latitudeDegrees, sizeof(GPS_fixPosition_t));

                GPS_handleFixEvent(currentRMCTime, currentRMCMilliSeconds, &fixTime, &fixPosition, parserResultDEFAULT.message);              

                /* Check RMC is valid */

                int64_t timeSincePPS = (int64_t)currentRMCTime * MILLISECONDS_IN_SECOND + (int64_t)currentRMCMilliSeconds - (int64_t)currentPPSTime * MILLISECONDS_IN_SECOND - (int64_t)currentPPSMilliSeconds;
                            
                if (parserResultRMC.milliseconds == 0 && timeSincePPS < MILLISECONDS_IN_SECOND) {

                    validRMC += 1;

                    if (validRMC > MINIMUM_VALID_RMC_TO_SET_TIME) {

                        uint32_t timestampInRMC;

                        GPSUtilities_getTime(&parserResultRMC, &timestampInRMC);

                        timeToBeSetOnNextPPS = timestampInRMC + 1;

                    }

                } else {

                    validRMC = 0;

                    validPPS = 0;

                    timeToBeSetOnNextPPS = 0;

                }

            }

            receivedRMC = false;

        }

        if (receivedPPS) {

            /* Check PPS is valid */

            int64_t timeSinceRMC = (int64_t)currentPPSTime * MILLISECONDS_IN_SECOND + (int64_t)currentPPSMilliSeconds - (int64_t)currentRMCTime * MILLISECONDS_IN_SECOND - (int64_t)currentRMCMilliSeconds;

            if (timeSinceRMC < MILLISECONDS_IN_SECOND) {

                validPPS += 1;

                if (validPPS > MINIMUM_VALID_PPS_TO_SET_TIME && timeToBeSetOnNextPPS > 0) {

                    /* Calculate clock difference and call handler */

                    int64_t timeDifference = (int64_t)currentPPSTime * MILLISECONDS_IN_SECOND + (int64_t)currentPPSMilliSeconds - (int64_t)timeToBeSetOnNextPPS * MILLISECONDS_IN_SECOND;

                    /* Calculate the actual clock frequency */

                    uint32_t expectedTimerCount = (previousTimerCount + clockFrequency) % TIMER_OVERFLOW_INTERVAL;

                    uint32_t measuredClockFrequency = clockFrequency;

                    if (expectedTimerCount > currentTimerCount) {

                        if (expectedTimerCount - currentTimerCount < TIMER_OVERFLOW_INTERVAL / 2) {

                            measuredClockFrequency -= expectedTimerCount - currentTimerCount;
                        
                        } else {

                            measuredClockFrequency += currentTimerCount + TIMER_OVERFLOW_INTERVAL - expectedTimerCount;

                        }

                    } else {
                        
                        if (currentTimerCount - expectedTimerCount < TIMER_OVERFLOW_INTERVAL / 2) {

                            measuredClockFrequency += currentTimerCount - expectedTimerCount;
                        
                        } else {

                            measuredClockFrequency -= expectedTimerCount + TIMER_OVERFLOW_INTERVAL - currentTimerCount;

                        }                        
                        
                    }

                    /* Call handler and return */

                    GPS_handleSetTime(timeToBeSetOnNextPPS, 0, timeDifference, measuredClockFrequency);

                    return GPS_SUCCESS;

                }

            } else {

                validRMC = 0;

                validPPS = 0;

                timeToBeSetOnNextPPS = 0;

            }

            /* Call the handler */

            GPS_handlePPSEvent(currentPPSTime, currentPPSMilliSeconds);

            receivedPPS = false;

        }

        EMU_EnterEM1();

        GPS_handleGetTime(&currentTime, NULL);

    }

    if (cancelledBySwitch) return GPS_CANCELLED_BY_SWITCH;

    if (cancelledByMagneticSwitch) return GPS_CANCELLED_BY_MAGNETIC_SWITCH;

    return GPS_TIMEOUT;

}

void GPS_cancelTimeSetting(GPS_fixCancellationReason_t reason) {

    if (reason == GPS_CANCEL_BY_SWITCH) cancelledBySwitch = true;

    if (reason == GPS_CANCEL_BY_MAGNETIC_SWITCH) cancelledByMagneticSwitch = true;

}