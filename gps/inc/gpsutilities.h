/****************************************************************************
 * gpsutilities.h
 * openacousticdevices.info
 * July 2020
 *****************************************************************************/
 
#ifndef GPSUTILITIES_H
#define GPSUTILITIES_H

#include "nmeaparser.h"

void GPSUtilities_getTime(NMEA_parserResultRMC_t *result, uint32_t *time);

void GPSUtilities_getLatitude(NMEA_parserResultRMC_t *result, float *latitude);

void GPSUtilities_getLongitude(NMEA_parserResultRMC_t *result, float *longitude);

void GPSUtilities_getFractionalYearInRadians(NMEA_parserResultRMC_t *result, float *gamma);

bool GPSUtilities_calculateSunsetAndSunrise(float gamma, float latitude, float longitude, uint16_t *sunriseMinutes, uint16_t *sunsetMinutes);

#endif /* GPSUTILITIES_H */
