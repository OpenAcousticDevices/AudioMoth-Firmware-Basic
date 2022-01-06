/****************************************************************************
 * butterworth.h
 * openacousticdevices.info
 * May 2020
 *****************************************************************************/

#ifndef __BUTTERWORTH_H
#define __BUTTERWORTH_H

#include <stdint.h>

typedef struct {
    float xv[3];
    float yv[3];
} BW_filter_t;

typedef struct {
    float gain;
    float yc[2];
} BW_filterCoefficients_t;

void Butterworth_designLowPassFilter(BW_filterCoefficients_t *filterCoefficients, uint32_t sampleRate, uint32_t freq);

void Butterworth_designHighPassFilter(BW_filterCoefficients_t *filterCoefficients, uint32_t sampleRate, uint32_t freq);

void Butterworth_designBandPassFilter(BW_filterCoefficients_t *filterCoefficients, uint32_t sampleRate, uint32_t freq1, uint32_t freq2);

void Butterworth_initialise(BW_filter_t *filter);

float Butterworth_applyLowPassFilter(float sample, BW_filter_t *filter, BW_filterCoefficients_t *filterCoefficients);

float Butterworth_applyBandPassFilter(float sample, BW_filter_t *filter, BW_filterCoefficients_t *filterCoefficients);

float Butterworth_applyHighPassFilter(float sample, BW_filter_t *filter, BW_filterCoefficients_t *filterCoefficients);

#endif /* __BUTTERWORTH_H */
