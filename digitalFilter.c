/****************************************************************************
 * digitalFilter.c
 * openacousticdevices.info
 * April 2020
 *****************************************************************************/

#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <complex.h>

#include "digitalFilter.h"

/*  Useful macros */

#ifndef M_PI
#define M_PI            3.14159265358979323846f
#endif

#ifndef M_TWOPI
#define M_TWOPI         (2.0f * M_PI)
#endif

#define MIN(a, b)       ((a) < (b) ? (a) : (b))
#define MAX(a, b)       ((a) > (b) ? (a) : (b))

/* Filter design constants */

#define MAX_POLES       2

/* Filter global variables */

static float gain;

static float yc0, yc1;

static float xv0, xv1, xv2;
static float yv0, yv1, yv2;

DF_filterType_t filterType;

/* Filter design variables */

static complex float spoles[MAX_POLES];
static complex float szeros[MAX_POLES];

static complex float zpoles[MAX_POLES];
static complex float zzeros[MAX_POLES];

static complex float topcoeffs[MAX_POLES + 1];
static complex float botcoeffs[MAX_POLES + 1];

/* Static filter design functions */

static complex float blt(complex float pz) {

    return (2.0f + pz) / (2.0f - pz);

}

static void expand(complex float pz[], uint32_t npz, complex float coeffs[]) {

    coeffs[0] = 1.0f;

    for (uint32_t i=0; i < npz; i += 1) {

        coeffs[i + 1] = 0.0f;

    }

    for (uint32_t i=0; i < npz; i += 1) {

        complex float nw = -pz[i];

        for (uint32_t i = npz; i >= 1; i -= 1) {

            coeffs[i] = (nw * coeffs[i]) + coeffs[i-1];

        }

        coeffs[0] = nw * coeffs[0];

    }

}

static complex float eval(complex float coeffs[], int npz, complex float z) {

   complex float sum = 0.0f;

   for (int32_t i = npz; i >= 0; i -= 1) {

       sum = (sum * z) + coeffs[i];

   }

   return sum;

}

static complex float evaluate(complex float topco[], int nz, complex float botco[], int np, complex float z) {

    return eval(topco, nz, z) / eval(botco, np, z);

}

/* Static filter function */

static inline float applyHighPassFilter(float sample) {

    xv0 = xv1;
    xv1 = sample * gain;

    yv0 = yv1;
    yv1 = xv1 - xv0 + yc0 * yv0;

    return yv1;

}

static inline float applyBandPassFilter(float sample) {

    xv0 = xv1;
    xv1 = xv2;
    xv2 = sample * gain;

    yv0 = yv1;
    yv1 = yv2;
    yv2 = xv2 - xv0 + yc0 * yv0 + yc1 * yv1;

    return yv2;

}

static inline void writeFilteredOutput(int16_t *dest, uint32_t index, float filterOutput, bool *exceededApplitudeThreshold, uint32_t amplitudeThreshold) {

    /* Apply output range limits */

    if (filterOutput > INT16_MAX) {

        filterOutput = INT16_MAX;

    } else if (filterOutput < -INT16_MAX) {

        filterOutput = -INT16_MAX;

    }

    /* Check if amplitude threshold is exceeded */

    if (fabsf(filterOutput) >= amplitudeThreshold) {

        *exceededApplitudeThreshold = true;

    }

    /* Write the output value */

    dest[index]  = (int16_t)filterOutput;

}

/* General filter routine */

static bool filter(int16_t *source, int16_t *dest, uint32_t sampleRateDivider, uint32_t size, uint16_t amplitudeThreshold) {

    uint32_t index = 0;

    bool exceededApplitudeThreshold = false;

    for (uint32_t i = 0; i < size; i += sampleRateDivider) {

        float sample = 0.0f;

        for (uint32_t j = 0; j < sampleRateDivider; j += 1) {

            sample += source[i + j];

        }

        float filterOutput;

        if (filterType == DF_HIGH_PASS_FILTER) {

            filterOutput = applyHighPassFilter(sample);

        } else {

            filterOutput = applyBandPassFilter(sample);

        }

        writeFilteredOutput(dest, index, filterOutput, &exceededApplitudeThreshold, amplitudeThreshold);

        index += 1;

    }

    return exceededApplitudeThreshold;

}

/* Fast filter routine for when 250kHz and 384kHz and sampleRateDivider is not needed */

static bool fastFilter(int16_t *source, int16_t *dest, uint32_t size, uint16_t amplitudeThreshold) {

    bool exceededAmplitudeThreshold = false;

    if (filterType == DF_HIGH_PASS_FILTER) {

        for (uint32_t index = 0; index < size; index += 1) {

            float sample = source[index];

            float filterOutput = applyHighPassFilter(sample);

            writeFilteredOutput(dest, index, filterOutput, &exceededAmplitudeThreshold, amplitudeThreshold);

        }

    } else {

        for (uint32_t index = 0; index < size; index += 1) {

            float sample = source[index];

            float filterOutput = applyBandPassFilter(sample);

            writeFilteredOutput(dest, index, filterOutput, &exceededAmplitudeThreshold, amplitudeThreshold);

        }

    }

    return exceededAmplitudeThreshold;

}

/* Reset the filter */

void DigitalFilter_reset() {

    xv0 = 0.0f;
    xv1 = 0.0f;
    xv2 = 0.0f;

    yv0 = 0.0f;
    yv1 = 0.0f;
    yv2 = 0.0f;

}

/* Update filter gain */

void DigitalFilter_applyAdditionalGain(float g) {

    gain *= g;

}

/* Apply digital filter */

bool DigitalFilter_filter(int16_t *source, int16_t *dest, uint32_t sampleRateDivider, uint32_t size, uint16_t amplitudeThreshold) {

    if (sampleRateDivider == 1) {

        return fastFilter(source, dest, size, amplitudeThreshold);

    } else {

        return filter(source, dest, sampleRateDivider, size, amplitudeThreshold);

    }

}

/* Design filters */

static void designFilter(uint32_t sampleRate, DF_filterType_t type, uint32_t freq1, uint32_t freq2) {

    /* Set filter type */

    filterType = type;

    /* Normalise frequencies */

    float alpha1 = (float)freq1 / (float)sampleRate;
    float alpha2 = (float)freq2 / (float)sampleRate;

    float warped_alpha1 = tanf(M_PI * alpha1) / M_PI;
    float warped_alpha2 = tanf(M_PI * alpha2) / M_PI;

    if (filterType == DF_HIGH_PASS_FILTER) {

        /* Calculate S poles */

        spoles[0] = -1.0f + 0.0f * I;

        /* Normalise S plane - 1 pole and 1 zero */

        spoles[0] = M_TWOPI * warped_alpha1 / spoles[0];

        szeros[0] = 0.0f;

        /* Calculate Z plane - 1 pole and 1 zero */

        zpoles[0] = blt(spoles[0]);
        zzeros[0] = blt(szeros[0]);

        /* Calculate top and bottom coefficients */

        expand(zzeros, 1, topcoeffs);
        expand(zpoles, 1, botcoeffs);

        /* Calculate Y coefficients */

        yc0 = -(crealf(botcoeffs[0]) / crealf(botcoeffs[1]));

        /* Calculate gain */

        complex float hf_gain = evaluate(topcoeffs, 1, botcoeffs, 1, -1.0f);

        gain = 1.0f / hypotf(crealf(hf_gain), cimagf(hf_gain));

    } else {

        /* Calculate S poles */

        spoles[0] = -1.0f + 0.0f * I;

        /* Normalise S plane - 2 poles and 1 zero */

        float w1 = M_TWOPI * warped_alpha1;
        float w2 = M_TWOPI * warped_alpha2;

        float w0 = sqrtf(w1 * w2);
        float bw = w2 - w1;

        complex float hba = 0.5f * bw * spoles[0];

        complex float temp = csqrt(1.0f - (w0 / hba) * (w0 / hba));

        spoles[0] = hba * (1.0f + temp);
        spoles[1] = hba * (1.0f - temp);

        szeros[0] = 0.0f;

        /* Calculate Z plane - 2 poles and 2 zeros */

        zpoles[0] = blt(spoles[0]);
        zpoles[1] = blt(spoles[1]);

        zzeros[0] = blt(szeros[0]);
        zzeros[1] = -1.0f;

        /* Calculate top and bottom coefficients */

        expand(zzeros, 2, topcoeffs);
        expand(zpoles, 2, botcoeffs);

        /* Calculate Y coefficients */

        yc0 = -(crealf(botcoeffs[0]) / crealf(botcoeffs[2]));
        yc1 = -(crealf(botcoeffs[1]) / crealf(botcoeffs[2]));

        /* Calculate gain */

        float theta = M_TWOPI * 0.5f * (alpha1 + alpha2);

        complex float expTheta = cosf(theta) + sinf(theta) * I;

        complex float fc_gain = evaluate(topcoeffs, 2, botcoeffs, 2, expTheta);

        gain = 1.0f / hypotf(crealf(fc_gain), cimagf(fc_gain));

    }

}

/* Design filters */

void DigitalFilter_designHighPassFilter(uint32_t sampleRate, uint32_t freq) {

    freq = MIN(sampleRate / 2, freq);

    designFilter(sampleRate, DF_HIGH_PASS_FILTER, freq, 0);

}

void DigitalFilter_designBandPassFilter(uint32_t sampleRate, uint32_t freq1, uint32_t freq2) {

    freq1 = MIN(sampleRate / 2, freq1);
    freq2 = MIN(sampleRate / 2, freq2);

    if (freq1 >= freq2) {

        yc0 = -1.0f;

        gain = 0.0f;

        filterType = DF_HIGH_PASS_FILTER;

    } else if (freq2 == sampleRate / 2) {

        designFilter(sampleRate, DF_HIGH_PASS_FILTER, freq1, 0);

    } else {

        designFilter(sampleRate, DF_BAND_PASS_FILTER, freq1, freq2);

    }

}

/* Read back filter setting */

void DigitalFilter_readSettings(float *gainPtr, float *yc0Ptr, float *yc1Ptr, DF_filterType_t *filterTypePtr) {

    *gainPtr = gain;

    *yc0Ptr = yc0;

    *yc1Ptr = yc1;

    *filterTypePtr = filterType;

}



