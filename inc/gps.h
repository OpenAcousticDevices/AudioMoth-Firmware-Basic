/****************************************************************************
 * gps.h
 * openacousticdevices.info
 * July 2021
 *****************************************************************************/
 
#ifndef __GPS_H
#define __GPS_H

#include <stdbool.h>

/* GPS constant */

#define GPS_TICK_EVENTS_PER_SECOND              12

/* Fix state enumeration */

typedef enum {GPS_CANCEL_BY_SWITCH, GPS_CANCEL_BY_MAGNETIC_SWITCH} GPS_fixCancellationReason_t;

typedef enum {GPS_SUCCESS, GPS_CANCELLED_BY_SWITCH, GPS_CANCELLED_BY_MAGNETIC_SWITCH, GPS_TIMEOUT} GPS_fixResult_t;

#pragma pack(push, 1)

typedef struct {
    uint8_t hours;
    uint8_t minutes;
    uint8_t seconds;
    uint16_t milliseconds;
    uint8_t day;
    uint8_t month;
    uint16_t year;
} GPS_fixTime_t;

typedef struct {
    uint8_t latitudeDegrees;
    uint8_t latitudeMinutes;
    uint16_t latitudeTenThousandths;
    char latitudeDirection;
    uint8_t longitudeDegrees;
    uint8_t longitudeMinutes;
    uint16_t longitudeTenThousandths;
    char longitudeDirection;
} GPS_fixPosition_t;

#pragma pack(pop)

/* External time handlers */

extern void GPS_handleSetTime(uint32_t time, uint32_t milliseconds, int64_t timeDifference, uint32_t measuredClockFrequency);

extern void GPS_handleGetTime(uint32_t *time, uint32_t *milliseconds);

/* External interrupt handlers */

extern void GPS_handleTickEvent();

extern void GPS_handlePPSEvent(uint32_t time, uint32_t milliseconds);

extern void GPS_handleFixEvent(uint32_t time, uint32_t milliseconds, GPS_fixTime_t *fixTime, GPS_fixPosition_t *fixPosition, char *message);

extern void GPS_handleMessageEvent(uint32_t time, uint32_t milliseconds, char *message);

extern void GPS_handleMagneticSwitchInterrupt(void);

/* Public functions */

void GPS_powerUpGPS(void);

void GPS_powerDownGPS(void);

void GPS_enableGPSInterface(void);

void GPS_disableGPSInterface(void);

void GPS_enableMagneticSwitch(void);

void GPS_disableMagneticSwitch(void);

bool GPS_isMagneticSwitchClosed(void);

GPS_fixResult_t GPS_setTimeFromGPS(uint32_t timeout);

void GPS_cancelTimeSetting(GPS_fixCancellationReason_t reason);

#endif /* __GPS_H */