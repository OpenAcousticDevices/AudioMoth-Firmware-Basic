/****************************************************************************
 * sunrise.h
 * openacousticdevices.info
 * July 2020
 *****************************************************************************/

#ifndef SUNRISE_H_
#define SUNRISE_H_

bool Sunrise_calculateFromDate(uint16_t year, uint8_t month, uint8_t day, float latitude, float longitude, uint16_t *sunriseMinutes, uint16_t *sunsetMinutes);

bool Sunrise_calculateFromUnix(uint32_t currentTime, float latitude, float longitude, uint16_t *sunriseMinutes, uint16_t *sunsetMinutes);

#endif /* SUNRISE_H_ */
