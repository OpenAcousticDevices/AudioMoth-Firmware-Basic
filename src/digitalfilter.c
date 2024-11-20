/****************************************************************************
 * digitalfilter.c
 * openacousticdevices.info
 * April 2020
 *****************************************************************************/

#include <math.h>
#include <float.h>
#include <stdint.h>
#include <stdbool.h>
#include <complex.h>

#include "digitalfilter.h"

/* Filter design constants */

#ifndef M_PI
#define M_PI                                    3.14159265358979323846f
#endif

#ifndef M_TWOPI
#define M_TWOPI                                 (2.0f * M_PI)
#endif

#define MAX_POLES                               2

/* Goertzel filter constants */

#define MAXIMUM_HAMMING_WINDOW_LENGTH           1024

#define MINIMUM_NUMBER_OF_ITERATIONS            16

/* Useful macros */

#define MIN(a, b)                               ((a) < (b) ? (a) : (b))
#define MAX(a, b)                               ((a) > (b) ? (a) : (b))

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

/* Amplitude threshold variable */

static uint16_t amplitudeThreshold;

/* Goertzel filter variables */

static float hammingWindow[MAXIMUM_HAMMING_WINDOW_LENGTH];

static uint32_t goertzelFilterWindowLength;

static float goertzelFilterThreshold;

static float goertzelFilterConstant;

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

/* General filter routine */

static bool filter(int16_t *source, int16_t *dest, uint32_t sampleRateDivider, uint32_t size) {

    uint32_t index = 0;

    bool exceededThreshold = false;

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

        /* Apply output range limits */

        if (filterOutput > INT16_MAX) {

            filterOutput = INT16_MAX;

        } else if (filterOutput < -INT16_MAX) {

            filterOutput = -INT16_MAX;

        }

        /* Check if amplitude threshold is exceeded */

        if (fabsf(filterOutput) >= amplitudeThreshold) {

            exceededThreshold = true;

        }

        /* Write the output value */

        dest[index++] = (int16_t)filterOutput;

    }

    return exceededThreshold;

}

/* Fast filter routine for when 250kHz and 384kHz and sampleRateDivider is not needed */

static bool fastFilterWithGoertzelFilterThreshold(int16_t *source, int16_t *dest, uint32_t size) {

    uint32_t index = 0;

    bool exceededThreshold = false;

    if (filterType == DF_HIGH_PASS_FILTER) {

        for (uint32_t i = 0; i < size / goertzelFilterWindowLength; i += 1) {

            float d1 = 0.0f;

            float d2 = 0.0f;

            uint32_t hammingIndex = 0;

            for (uint32_t j = 0; j < goertzelFilterWindowLength / MINIMUM_NUMBER_OF_ITERATIONS; j += 1) {

                for (uint32_t k = 0; k < MINIMUM_NUMBER_OF_ITERATIONS; k += 1) {

                    float sample = source[index];

                    float filterOutput = applyHighPassFilter(sample);

                    /* Apply output range limits */

                    if (filterOutput > INT16_MAX) {

                        filterOutput = INT16_MAX;

                    } else if (filterOutput < -INT16_MAX) {

                        filterOutput = -INT16_MAX;

                    }

                    /* Update Goertzel filter */

                    float y = hammingWindow[hammingIndex++] * filterOutput + goertzelFilterConstant * d1 - d2;

                    d2 = d1;

                    d1 = y;

                    /* Write the output value */

                    dest[index++] = (int16_t)filterOutput;
                    
                }

            }

            float squaredMagnitude = d1 * d1 + d2 * d2 - goertzelFilterConstant * d1 * d2;

            if (squaredMagnitude > goertzelFilterThreshold) exceededThreshold = true;

        }

    } else {

        for (uint32_t i = 0; i < size / goertzelFilterWindowLength; i += 1) {

            float d1 = 0.0f;

            float d2 = 0.0f;

            uint32_t hammingIndex = 0;

            for (uint32_t j = 0; j < goertzelFilterWindowLength / MINIMUM_NUMBER_OF_ITERATIONS; j += 1) {

                for (uint32_t k = 0; k < MINIMUM_NUMBER_OF_ITERATIONS; k += 1) {

                    float sample = source[index];

                    float filterOutput = applyBandPassFilter(sample);

                    /* Apply output range limits */

                    if (filterOutput > INT16_MAX) {

                        filterOutput = INT16_MAX;

                    } else if (filterOutput < -INT16_MAX) {

                        filterOutput = -INT16_MAX;

                    }
                
                    /* Update Goertzel filter */

                    float y = hammingWindow[hammingIndex++] * filterOutput + goertzelFilterConstant * d1 - d2;

                    d2 = d1;

                    d1 = y;

                    /* Write the output value */

                    dest[index++] = (int16_t)filterOutput;

                }

            }

            float squaredMagnitude = d1 * d1 + d2 * d2 - goertzelFilterConstant * d1 * d2;

            if (squaredMagnitude > goertzelFilterThreshold) exceededThreshold = true;

        }
    
    }

    return exceededThreshold;

}

static bool fastFilterWithAmplitudeThreshold(int16_t *source, int16_t *dest, uint32_t size) {

    uint32_t index = 0;

    bool exceededThreshold = false;

    if (filterType == DF_HIGH_PASS_FILTER) {

        for (uint32_t i = 0; i < size / MINIMUM_NUMBER_OF_ITERATIONS; i += 1) {

            for (uint32_t j = 0; j < MINIMUM_NUMBER_OF_ITERATIONS; j += 1) {

                float sample = source[index];

                float filterOutput = applyHighPassFilter(sample);

                /* Apply output range limits */

                if (filterOutput > INT16_MAX) {

                    filterOutput = INT16_MAX;

                } else if (filterOutput < -INT16_MAX) {

                    filterOutput = -INT16_MAX;

                }

                /* Check if amplitude threshold is exceeded */

                if (fabsf(filterOutput) >= amplitudeThreshold) {

                    exceededThreshold = true;

                }

                /* Write the output value */

                dest[index++] = (int16_t)filterOutput;

            }

        }

    } else {

        for (uint32_t i = 0; i < size / MINIMUM_NUMBER_OF_ITERATIONS; i += 1) {

            for (uint32_t j = 0; j < MINIMUM_NUMBER_OF_ITERATIONS; j += 1) {

                float sample = source[index];

                float filterOutput = applyBandPassFilter(sample);

                /* Apply output range limits */

                if (filterOutput > INT16_MAX) {

                    filterOutput = INT16_MAX;

                } else if (filterOutput < -INT16_MAX) {

                    filterOutput = -INT16_MAX;

                }

                /* Check if amplitude threshold is exceeded */

                if (fabsf(filterOutput) >= amplitudeThreshold) {

                    exceededThreshold = true;

                }

                /* Write the output value */

                dest[index++] = (int16_t)filterOutput;

            }

        }

    }

    return exceededThreshold;

}

/* Reset the filter */

void DigitalFilter_reset() {

    xv0 = 0.0f;
    xv1 = 0.0f;
    xv2 = 0.0f;

    yv0 = 0.0f;
    yv1 = 0.0f;
    yv2 = 0.0f;

    amplitudeThreshold = 0;

    goertzelFilterThreshold = 0.0f;

}

/* Update filter gain */

void DigitalFilter_setAdditionalGain(float g) {

    gain *= g;

}

/* Apply digital filter */

bool DigitalFilter_applyFilter(int16_t *source, int16_t *dest, uint32_t sampleRateDivider, uint32_t size) {

    if (sampleRateDivider == 1) {

        if (goertzelFilterThreshold > 0.0f) {

            return fastFilterWithGoertzelFilterThreshold(source, dest, size);

        } else {

            return fastFilterWithAmplitudeThreshold(source, dest, size);

        }

    } else {

        return filter(source, dest, sampleRateDivider, size);

    }

}

bool DigitalFilter_applyFrequencyTrigger(int16_t *source, uint32_t size) {

    uint32_t index = 0;

    for (uint32_t i = 0; i < size / goertzelFilterWindowLength; i += 1) {

        float d1 = 0.0f;

        float d2 = 0.0f;

        uint32_t hammingIndex = 0;

        for (uint32_t j = 0; j < goertzelFilterWindowLength / MINIMUM_NUMBER_OF_ITERATIONS; j += 1) {

            for (uint32_t k = 0; k < MINIMUM_NUMBER_OF_ITERATIONS; k += 1) {

                float y = hammingWindow[hammingIndex++] * (float)source[index++] + goertzelFilterConstant * d1 - d2;
    
                d2 = d1;
    
                d1 = y;

            }

        }

        float squaredMagnitude = d1 * d1 + d2 * d2 - goertzelFilterConstant * d1 * d2;

        if (squaredMagnitude > goertzelFilterThreshold) return true;

    }

    return false;

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

/* Set threshold options */

void DigitalFilter_setAmplitudeThreshold(uint16_t threshold) {

    amplitudeThreshold = threshold;

}

void DigitalFilter_setFrequencyTrigger(uint32_t windowLength, uint32_t sampleRate, uint32_t frequency, float percentageThreshold) {

    goertzelFilterThreshold = 0.0f;

    goertzelFilterWindowLength = windowLength;

    for (uint32_t i = 0; i < goertzelFilterWindowLength; i += 1) {

        hammingWindow[i] = 0.54f - 0.46f * cosf(M_TWOPI * (float)i / (float)(goertzelFilterWindowLength - 1));

        goertzelFilterThreshold += hammingWindow[i];

    }

    goertzelFilterThreshold *= (float)INT16_MAX / 2.0f;

    goertzelFilterThreshold = percentageThreshold >= 100.0f ? FLT_MAX : goertzelFilterThreshold * goertzelFilterThreshold * percentageThreshold / 100.0f * percentageThreshold / 100.0f;

    goertzelFilterConstant = 2.0f * cosf(M_TWOPI * (float)frequency / (float)sampleRate);

}

/* Read back filter setting */

void DigitalFilter_readSettings(float *gainPtr, float *yc0Ptr, float *yc1Ptr, DF_filterType_t *filterTypePtr) {

    *gainPtr = gain;

    *yc0Ptr = yc0;

    *yc1Ptr = yc1;

    *filterTypePtr = filterType;

}
