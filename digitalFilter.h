/****************************************************************************
 * digitalFilter.h
 * openacousticdevices.info
 * April 2020
 *****************************************************************************/

#ifndef __DIGITAL_FILTER_H
#define __DIGITAL_FILTER_H

#include <stdint.h>
#include <stdbool.h>

/* Digital filter enumeration */

typedef enum {DF_BAND_PASS_FILTER, DF_HIGH_PASS_FILTER} DF_filterType_t;

/* Apply filters */

void DigitalFilter_reset();

void DigitalFilter_applyAdditionalGain(float gain);

bool DigitalFilter_filter(int16_t *source, int16_t *dest, uint32_t sampleRateDivider, uint32_t size, uint16_t amplitudeThreshold);

/* Design filters */

void DigitalFilter_designHighPassFilter(uint32_t sampleRate, uint32_t freq);

void DigitalFilter_designBandPassFilter(uint32_t sampleRate, uint32_t freq1, uint32_t freq2);

/* Read back filter setting */

void DigitalFilter_readSettings(float *gain, float *yc0, float *yc1, DF_filterType_t *filterType);

#endif /* __DIGITAL_FILTER_H */
