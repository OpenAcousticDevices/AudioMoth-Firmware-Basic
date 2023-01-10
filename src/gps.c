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
#include "gpsinterface.h"

/* Hardware constants */

#define MAGNETIC_SWITCH_GPIOPORT                gpioPortB
#define MAGNETIC_SWITCH_PIN                     9

#define GPS_ENABLE_N_GPIOPORT                   gpioPortA
#define GPS_ENABLE_N_PIN                        7

/* GPS fix constants */

#define MINIMUM_VALID_PPS_TO_SET_TIME           4
#define MINIMUM_VALID_RMC_TO_SET_TIME           4

/* Useful time constants */

#define MILLISECONDS_IN_SECOND                  1000

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

/* Timer variables */

static volatile uint32_t currentTimerCount;

static volatile uint32_t previousTimerCount;

static volatile uint32_t currentTimerPeriod;

static volatile uint32_t currentTimerFrequency;

/* Current PPS and RMC variables */

static volatile uint32_t currentPPSTime;

static volatile uint32_t currentPPSMilliSeconds;

static volatile uint32_t currentRMCTime;

static volatile uint32_t currentRMCMilliSeconds;

static volatile uint32_t currentMessageTime;

static volatile uint32_t currentMessageMilliSeconds;

/* Interrupt handler for magnetic switch */

void GPIO_ODD_IRQHandler(void) {

    /* Get the interrupt mask */

    uint32_t interruptMask = GPIO_IntGet();

    /* Clear the interrupt */

    GPIO_IntClear(interruptMask);

    /* Handle the interrupt */

    if (interruptMask & (1 << MAGNETIC_SWITCH_PIN)) GPS_handleMagneticSwitchInterrupt();

}

/* Interrupt handler for RX */

inline void GPSInterface_handleReceivedByte(uint8_t byte) {

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

/* Interrupt handler for GPS PPS and tick */

inline void GPSInterface_handlePulsePerSecond(uint32_t counter, uint32_t counterPeriod, uint32_t counterFrequency) {

    GPS_handleGetTime((uint32_t*)&currentPPSTime, (uint32_t*)&currentPPSMilliSeconds);

    previousTimerCount = currentTimerCount;

    currentTimerPeriod = counterPeriod;

    currentTimerFrequency = counterFrequency;

    currentTimerCount = counter;

    receivedPPS = true;

}

inline void GPSInterface_handleTick() {

    tick = true;

    WDOG_Feed();

}

/* Public functions */

void GPS_powerUpGPS() {

    GPIO_PinModeSet(GPS_ENABLE_N_GPIOPORT, GPS_ENABLE_N_PIN, gpioModePushPull, 0);

}

void GPS_powerDownGPS() {

    GPIO_PinModeSet(GPS_ENABLE_N_GPIOPORT, GPS_ENABLE_N_PIN, gpioModeDisabled, 0);

}

void GPS_enableGPSInterface() {

    GPSInterface_enable(GPS_TICK_EVENTS_PER_SECOND);

}

void GPS_disableGPSInterface() {

    GPSInterface_disable();

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

                    uint32_t expectedTimerCount = (previousTimerCount + currentTimerFrequency) % currentTimerPeriod;

                    uint32_t measuredTimerFrequency = currentTimerFrequency;

                    if (expectedTimerCount > currentTimerCount) {

                        if (expectedTimerCount - currentTimerCount < currentTimerPeriod / 2) {

                            measuredTimerFrequency -= expectedTimerCount - currentTimerCount;
                        
                        } else {

                            measuredTimerFrequency += currentTimerCount + currentTimerPeriod - expectedTimerCount;

                        }

                    } else {
                        
                        if (currentTimerCount - expectedTimerCount < currentTimerPeriod / 2) {

                            measuredTimerFrequency += currentTimerCount - expectedTimerCount;
                        
                        } else {

                            measuredTimerFrequency -= expectedTimerCount + currentTimerPeriod - currentTimerCount;

                        }                        
                        
                    }

                    /* Call handler and return */

                    GPS_handleSetTime(timeToBeSetOnNextPPS, 0, timeDifference, measuredTimerFrequency);

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