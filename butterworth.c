/****************************************************************************
 * butterworth.c
 * openacousticdevices.info
 * May 2020
 *****************************************************************************/

#include <math.h>
#include <stdint.h>
#include <complex.h>

#include "butterworth.h"

/* Maths constants */

#ifndef M_PI
#define M_PI            3.14159265358979323846f
#endif

#ifndef M_TWOPI
#define M_TWOPI         (2.0f * M_PI)
#endif

/* Filter design constants */

#define MAX_POLES       2

/* Filter design variables */

static complex float spoles[MAX_POLES];
static complex float szeros[MAX_POLES];

static complex float zpoles[MAX_POLES];
static complex float zzeros[MAX_POLES];

static complex float topcoeffs[MAX_POLES + 1];
static complex float botcoeffs[MAX_POLES + 1];

typedef enum {BW_LOW_PASS_FILTER, BW_BAND_PASS_FILTER, BW_HIGH_PASS_FILTER} BW_filterType_t;

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

static void designFilter(BW_filterType_t filterType, BW_filterCoefficients_t *filterCoefficients, uint32_t sampleRate, uint32_t freq1, uint32_t freq2) {

    /* Normalise frequencies */

    float alpha1 = (float)freq1 / (float)sampleRate;
    float alpha2 = (float)freq2 / (float)sampleRate;

    float warped_alpha1 = tanf(M_PI * alpha1) / M_PI;
    float warped_alpha2 = tanf(M_PI * alpha2) / M_PI;

    if (filterType == BW_LOW_PASS_FILTER) {

        /* Calculate S poles */

        spoles[0] = -1.0f + 0.0f * I;

        /* Normalise S plane - 1 pole and 0 zero */

        spoles[0] = M_TWOPI * warped_alpha1 * spoles[0];

        /* Calculate Z plane - 1 pole and 1 zero */

        zpoles[0] = blt(spoles[0]);
        zzeros[0] = -1.0f;

        /* Calculate top and bottom coefficients */

        expand(zzeros, 1, topcoeffs);
        expand(zpoles, 1, botcoeffs);        

        /* Calculate Y coefficients */

        filterCoefficients->yc[0] = -(crealf(botcoeffs[0]) / crealf(botcoeffs[1]));

        /* Calculate gain */

        complex float dc_gain = evaluate(topcoeffs, 1, botcoeffs, 1, 1.0f);

        filterCoefficients->gain = 1.0f / hypotf(crealf(dc_gain), cimagf(dc_gain));

    } else if (filterType == BW_HIGH_PASS_FILTER) {

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

        filterCoefficients->yc[0] = -(crealf(botcoeffs[0]) / crealf(botcoeffs[1]));

        /* Calculate gain */

        complex float hf_gain = evaluate(topcoeffs, 1, botcoeffs, 1, -1.0f);

        filterCoefficients->gain = 1.0f / hypotf(crealf(hf_gain), cimagf(hf_gain));

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

        filterCoefficients->yc[0] = -(crealf(botcoeffs[0]) / crealf(botcoeffs[2]));
        filterCoefficients->yc[1] = -(crealf(botcoeffs[1]) / crealf(botcoeffs[2]));

        /* Calculate gain */

        float theta = M_TWOPI * 0.5f * (alpha1 + alpha2);

        complex float expTheta = cosf(theta) + sinf(theta) * I;

        complex float fc_gain = evaluate(topcoeffs, 2, botcoeffs, 2, expTheta);

        filterCoefficients->gain = 1.0f / hypotf(crealf(fc_gain), cimagf(fc_gain));

    }

}

/* Filter design function */

void Butterworth_designLowPassFilter(BW_filterCoefficients_t *filterCoefficients, uint32_t sampleRate, uint32_t freq) {

    designFilter(BW_LOW_PASS_FILTER, filterCoefficients, sampleRate, freq, 0);

}

void Butterworth_designBandPassFilter(BW_filterCoefficients_t *filterCoefficients, uint32_t sampleRate, uint32_t freq1, uint32_t freq2) {

    designFilter(BW_BAND_PASS_FILTER, filterCoefficients, sampleRate, freq1, freq2);

}

void Butterworth_designHighPassFilter(BW_filterCoefficients_t *filterCoefficients, uint32_t sampleRate, uint32_t freq) {

    designFilter(BW_HIGH_PASS_FILTER, filterCoefficients, sampleRate, freq, 0);

}

/* Initialise filter */

void Butterworth_initialise(BW_filter_t *filter) {

    for (uint32_t i = 0; i < 3; i += 1) {
        filter->xv[i] = 0.0f;
        filter->yv[i] = 0.0f;
    }

}

/* Apply filters */

float Butterworth_applyLowPassFilter(float sample, BW_filter_t *filter, BW_filterCoefficients_t *filterCoefficients) {

    filter->xv[0] = filter->xv[1];
    filter->xv[1] = sample * filterCoefficients->gain;

    filter->yv[0] = filter->yv[1];
    filter->yv[1] = filter->xv[0] + filter->xv[1] + filterCoefficients->yc[0] * filter->yv[0];

    return filter->yv[1];

}

float Butterworth_applyBandPassFilter(float sample, BW_filter_t *filter, BW_filterCoefficients_t *filterCoefficients) {

    filter->xv[0] = filter->xv[1];
    filter->xv[1] = filter->xv[2];
    filter->xv[2] = sample * filterCoefficients->gain;

    filter->yv[0] = filter->yv[1];
    filter->yv[1] = filter->yv[2];
    filter->yv[2] = filter->xv[2] - filter->xv[0] + filterCoefficients->yc[0] * filter->yv[0] + filterCoefficients->yc[1] * filter->yv[1];

    return filter->yv[2];

}

float Butterworth_applyHighPassFilter(float sample, BW_filter_t *filter, BW_filterCoefficients_t *filterCoefficients) {

    filter->xv[0] = filter->xv[1];
    filter->xv[1] = sample * filterCoefficients->gain;

    filter->yv[0] = filter->yv[1];
    filter->yv[1] = filter->xv[1] - filter->xv[0] + filterCoefficients->yc[0] * filter->yv[0];

    return filter->yv[1];

}


