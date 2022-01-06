/****************************************************************************
 * biquad.h
 * openacousticdevices.info
 * May 2020
 *****************************************************************************/

#ifndef __BIQUAD_H
#define __BIQUAD_H

#include <stdint.h>

typedef struct {
    float xv[3];
    float yv[3];
} BQ_filter_t;

typedef struct {
    float B0_A0;
    float B1_A0;
    float B2_A0;
    float A1_A0;
    float A2_A0;
} BQ_filterCoefficients_t;

void Biquad_designLowPassFilter(BQ_filterCoefficients_t *coefficients, uint32_t sampleRate, uint32_t frequency, float bandwidth);

void Biquad_designHighPassFilter(BQ_filterCoefficients_t *coefficients, uint32_t sampleRate, uint32_t frequency, float bandwidth);

void Biquad_designBandPassFilter(BQ_filterCoefficients_t *coefficients, uint32_t sampleRate, uint32_t frequency1, uint32_t frequency2);

void Biquad_designNotchFilter(BQ_filterCoefficients_t *coefficients, uint32_t sampleRate, uint32_t frequency1, uint32_t frequency2);

void Biquad_initialise(BQ_filter_t *filter);

float Biquad_applyFilter(float sample, BQ_filter_t *filter, BQ_filterCoefficients_t *filterCoefficients);

#endif /* __BIQUAD_H */
