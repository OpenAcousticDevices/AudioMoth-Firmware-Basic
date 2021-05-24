/****************************************************************************
 * main.c
 * openacousticdevices.info
 * June 2017
 *****************************************************************************/

#include <time.h>
#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#include "audioMoth.h"
#include "audioConfig.h"
#include "digitalFilter.h"

/* Useful time constants */

#define MILLISECONDS_IN_SECOND              1000

#define SECONDS_IN_MINUTE                   60
#define SECONDS_IN_HOUR                     (60 * SECONDS_IN_MINUTE)
#define SECONDS_IN_DAY                      (24 * SECONDS_IN_HOUR)

/* Useful type constants */

#define BITS_PER_BYTE                       8
#define UINT32_SIZE_IN_BITS                 32
#define UINT32_SIZE_IN_BYTES                4
#define UINT16_SIZE_IN_BYTES                2

/* Sleep and LED constants */

#define LOW_BATTERY_LED_FLASHES             10

#define SHORT_LED_FLASH_DURATION            100
#define LONG_LED_FLASH_DURATION             500

#define WAITING_LED_FLASH_DURATION          10
#define WAITING_LED_FLASH_INTERVAL          2000

#define DEFAULT_WAIT_INTERVAL               1000

/* SRAM buffer constants */

#define NUMBER_OF_BUFFERS                   8
#define NUMBER_OF_BYTES_IN_SAMPLE           2
#define EXTERNAL_SRAM_SIZE_IN_SAMPLES       (AM_EXTERNAL_SRAM_SIZE_IN_BYTES / NUMBER_OF_BYTES_IN_SAMPLE)
#define NUMBER_OF_SAMPLES_IN_BUFFER         (EXTERNAL_SRAM_SIZE_IN_SAMPLES / NUMBER_OF_BUFFERS)

/* DMA transfer constant */

#define MAXIMUM_SAMPLES_IN_DMA_TRANSFER     1024

/* Compression constants */

#define COMPRESSION_BUFFER_SIZE_IN_BYTES    512

/* File size constants */

#define MAXIMUM_FILE_NAME_LENGTH            32

#define MAXIMUM_WAV_FILE_SIZE               (UINT32_MAX - 1)

/* WAV header constant */

#define PCM_FORMAT                          1
#define RIFF_ID_LENGTH                      4
#define LENGTH_OF_ARTIST                    32
#define LENGTH_OF_COMMENT                   384

/* USB configuration constant */

#define MAX_START_STOP_PERIODS              5

/* Digital filter constant */

#define FILTER_FREQ_MULTIPLIER              100

/* DC filter constant */

#define LOW_DC_BLOCKING_FREQ                8
#define DEFAULT_DC_BLOCKING_FREQ            48

/* Supply monitor constant */

#define MINIMUM_SUPPLY_VOLTAGE              2800

/* Deployment ID constant */

#define DEPLOYMENT_ID_LENGTH                8

/* Audio configuration constants */

#define AUDIO_CONFIG_PULSE_INTERVAL         10
#define AUDIO_CONFIG_TIME_CORRECTION        134
#define AUDIO_CONFIG_TONE_TIMEOUT           250
#define AUDIO_CONFIG_PACKETS_TIMEOUT        30000

/* USB configuration constant */

#define USB_CONFIG_TIME_CORRECTION          26

/* EM4 wake constant */

#define EM4_WAKEUP_PERIOD                   43

/* Recording preparation constants */

#define PREPARATION_PERIOD_INCREMENT        250
#define INITIAL_PREPARATION_PERIOD          2000
#define MINIMUM_PREPARATION_PERIOD          1000
#define MAXIMUM_PREPARATION_PERIOD          30000

/* Energy saver mode constant */

#define ENERGY_SAVER_SAMPLE_RATE_THRESHOLD  48000

/* Useful macros */

#define FLASH_LED(led, duration) { \
    AudioMoth_set ## led ## LED(true); \
    AudioMoth_delay(duration); \
    AudioMoth_set ## led ## LED(false); \
}

#define FLASH_LED_AND_RETURN_ON_ERROR(fn) { \
    bool success = (fn); \
    if (success != true) { \
        FLASH_LED(Both, LONG_LED_FLASH_DURATION) \
        return SDCARD_WRITE_ERROR; \
    } \
}

#define RETURN_BOOL_ON_ERROR(fn) { \
    bool success = (fn); \
    if (success != true) { \
        return success; \
    } \
}

#define SAVE_SWITCH_POSITION_AND_POWER_DOWN(milliseconds) { \
    *previousSwitchPosition = switchPosition; \
    AudioMoth_powerDownAndWakeMilliseconds(milliseconds); \
}

#define SERIAL_NUMBER                           "%08X%08X"

#define FORMAT_SERIAL_NUMBER(src)               (unsigned int)*((uint32_t*)src + 1),  (unsigned int)*((uint32_t*)src)

#define ABS(a)                                  ((a) < (0) ? (-a) : (a))

#define MIN(a, b)                               ((a) < (b) ? (a) : (b))

#define MAX(a, b)                               ((a) > (b) ? (a) : (b))

#define ROUNDED_DIV(a, b)                       (((a) + (b/2)) / (b))

#define ROUNDED_UP_DIV(a, b)                    (((a) + (b) - 1) / (b))

#define ROUND_UP_TO_MULTIPLE(a, b)              (((a) + (b) - 1) & ~((b)-1))

/* Recording state enumeration */

typedef enum {RECORDING_OKAY, FILE_SIZE_LIMITED, SUPPLY_VOLTAGE_LOW, SWITCH_CHANGED, MICROPHONE_CHANGED, SDCARD_WRITE_ERROR} AM_recordingState_t;

/* Filter type enumeration */

typedef enum {NO_FILTER, LOW_PASS_FILTER, BAND_PASS_FILTER, HIGH_PASS_FILTER} AM_filterType_t;

/* Battery level display type */

typedef enum {BATTERY_LEVEL, NIMH_LIPO_BATTERY_VOLTAGE} AM_batteryLevelDisplayType_t;

/* WAV header */

#pragma pack(push, 1)

typedef struct {
    char id[RIFF_ID_LENGTH];
    uint32_t size;
} chunk_t;

typedef struct {
    chunk_t icmt;
    char comment[LENGTH_OF_COMMENT];
} icmt_t;

typedef struct {
    chunk_t iart;
    char artist[LENGTH_OF_ARTIST];
} iart_t;

typedef struct {
    uint16_t format;
    uint16_t numberOfChannels;
    uint32_t samplesPerSecond;
    uint32_t bytesPerSecond;
    uint16_t bytesPerCapture;
    uint16_t bitsPerSample;
} wavFormat_t;

typedef struct {
    chunk_t riff;
    char format[RIFF_ID_LENGTH];
    chunk_t fmt;
    wavFormat_t wavFormat;
    chunk_t list;
    char info[RIFF_ID_LENGTH];
    icmt_t icmt;
    iart_t iart;
    chunk_t data;
} wavHeader_t;

#pragma pack(pop)

static wavHeader_t wavHeader = {
    .riff = {.id = "RIFF", .size = 0},
    .format = "WAVE",
    .fmt = {.id = "fmt ", .size = sizeof(wavFormat_t)},
    .wavFormat = {.format = PCM_FORMAT, .numberOfChannels = 1, .samplesPerSecond = 0, .bytesPerSecond = 0, .bytesPerCapture = 2, .bitsPerSample = 16},
    .list = {.id = "LIST", .size = RIFF_ID_LENGTH + sizeof(icmt_t) + sizeof(iart_t)},
    .info = "INFO",
    .icmt = {.icmt.id = "ICMT", .icmt.size = LENGTH_OF_COMMENT, .comment = ""},
    .iart = {.iart.id = "IART", .iart.size = LENGTH_OF_ARTIST, .artist = ""},
    .data = {.id = "data", .size = 0}
};

/* USB configuration data structure */

#pragma pack(push, 1)

typedef struct {
    uint16_t startMinutes;
    uint16_t stopMinutes;
} startStopPeriod_t;

typedef struct {
    uint32_t time;
    uint8_t gain;
    uint8_t clockDivider;
    uint8_t acquisitionCycles;
    uint8_t oversampleRate;
    uint32_t sampleRate;
    uint8_t sampleRateDivider;
    uint16_t sleepDuration;
    uint16_t recordDuration;
    uint8_t enableLED;
    uint8_t activeStartStopPeriods;
    startStopPeriod_t startStopPeriods[MAX_START_STOP_PERIODS];
    int8_t timezoneHours;
    uint8_t enableLowVoltageCutoff;
    uint8_t disableBatteryLevelDisplay;
    int8_t timezoneMinutes;
    uint8_t disableSleepRecordCycle;
    uint32_t earliestRecordingTime;
    uint32_t latestRecordingTime;
    uint16_t lowerFilterFreq;
    uint16_t higherFilterFreq;
    uint16_t amplitudeThreshold;
    uint8_t requireAcousticConfiguration : 1;
    AM_batteryLevelDisplayType_t batteryLevelDisplayType : 1;
    uint8_t minimumTriggerDuration : 6;
    uint8_t enableAmplitudeThresholdDecibelScale : 1;
    uint8_t amplitudeThresholdDecibels : 7; 
    uint8_t enableAmplitudeThresholdPercentageScale : 1;
    uint8_t amplitudeThresholdPercentageMantissa : 4; 
    int8_t amplitudeThresholdPercentageExponent : 3; 
    uint8_t enableEnergySaverMode : 1; 
    uint8_t disable48HzDCBlockingFilter : 1;
} configSettings_t;

#pragma pack(pop)

static const configSettings_t defaultConfigSettings = {
    .time = 0,
    .gain = 2,
    .clockDivider = 4,
    .acquisitionCycles = 16,
    .oversampleRate = 1,
    .sampleRate = 384000,
    .sampleRateDivider = 8,
    .sleepDuration = 5,
    .recordDuration = 55,
    .enableLED = 1,
    .activeStartStopPeriods = 1,
    .startStopPeriods = {
        {.startMinutes = 000, .stopMinutes = 1440},
        {.startMinutes = 000, .stopMinutes = 1440},
        {.startMinutes = 000, .stopMinutes = 1440},
        {.startMinutes = 000, .stopMinutes = 1440},
        {.startMinutes = 000, .stopMinutes = 1440}
    },
    .timezoneHours = 0,
    .enableLowVoltageCutoff = 1,
    .disableBatteryLevelDisplay = 0,
    .timezoneMinutes = 0,
    .disableSleepRecordCycle = 0,
    .earliestRecordingTime = 0,
    .latestRecordingTime = 0,
    .lowerFilterFreq = 0,
    .higherFilterFreq = 0,
    .amplitudeThreshold = 0,
    .requireAcousticConfiguration = 0,
    .batteryLevelDisplayType = BATTERY_LEVEL,
    .minimumTriggerDuration = 0,
    .enableAmplitudeThresholdDecibelScale = 0,
    .amplitudeThresholdDecibels = 0,
    .enableAmplitudeThresholdPercentageScale = 0,
    .amplitudeThresholdPercentageMantissa = 0,
    .amplitudeThresholdPercentageExponent = 0,
    .enableEnergySaverMode = 0,
    .disable48HzDCBlockingFilter = 0
};

/* Persistent configuration data structure */

#pragma pack(push, 1)

typedef struct {
    uint8_t firmwareVersion[AM_FIRMWARE_VERSION_LENGTH];
    uint8_t firmwareDescription[AM_FIRMWARE_DESCRIPTION_LENGTH];
    configSettings_t configSettings;
} persistentConfigSettings_t;

#pragma pack(pop)

/* Functions to format header and configuration components */

static uint32_t formatDecibels(char *dest, uint32_t value) {

    if (value) return sprintf(dest, "-%lu dB", value);

    memcpy(dest, "0 dB", 4);

    return 4;

}

static uint32_t formatPercentage(char *dest, uint32_t mantissa, int32_t exponent) {

    uint32_t length = exponent < 0 ? 1 - exponent : 0;

    memcpy(dest, "0.0000", length);

    length += sprintf(dest + length, "%lu", mantissa);

    while (exponent-- > 0) dest[length++] = '0';

    dest[length++] = '%';

    return length;

}

/* Functions to set WAV header details and comment */

static void setHeaderDetails(wavHeader_t *wavHeader, uint32_t sampleRate, uint32_t numberOfSamples) {

    wavHeader->wavFormat.samplesPerSecond = sampleRate;
    wavHeader->wavFormat.bytesPerSecond = NUMBER_OF_BYTES_IN_SAMPLE * sampleRate;
    wavHeader->data.size = NUMBER_OF_BYTES_IN_SAMPLE * numberOfSamples;
    wavHeader->riff.size = NUMBER_OF_BYTES_IN_SAMPLE * numberOfSamples + sizeof(wavHeader_t) - sizeof(chunk_t);

}

static void setHeaderComment(wavHeader_t *wavHeader, configSettings_t *configSettings, uint32_t currentTime, uint8_t *serialNumber, uint8_t *deploymentID, uint8_t *defaultDeploymentID, AM_extendedBatteryState_t extendedBatteryState, int32_t temperature, bool externalMicrophone, AM_recordingState_t recordingState, AM_filterType_t filterType) {

    time_t rawTime = currentTime + configSettings->timezoneHours * SECONDS_IN_HOUR + configSettings->timezoneMinutes * SECONDS_IN_MINUTE;

    struct tm *time = gmtime(&rawTime);

    /* Format artist field */

    char *artist = wavHeader->iart.artist;

    sprintf(artist, "AudioMoth " SERIAL_NUMBER, FORMAT_SERIAL_NUMBER(serialNumber));

    /* Format comment field */

    char *comment = wavHeader->icmt.comment;

    comment += sprintf(comment, "Recorded at %02d:%02d:%02d %02d/%02d/%04d (UTC", time->tm_hour, time->tm_min, time->tm_sec, time->tm_mday, 1 + time->tm_mon, 1900 + time->tm_year);

    int8_t timezoneHours = configSettings->timezoneHours;

    int8_t timezoneMinutes = configSettings->timezoneMinutes;

    if (timezoneHours < 0) {

        comment += sprintf(comment, "%d", timezoneHours);

    } else if (timezoneHours > 0) {

        comment += sprintf(comment, "+%d", timezoneHours);

    } else {

        if (timezoneMinutes < 0) comment += sprintf(comment, "-%d", timezoneHours);

        if (timezoneMinutes > 0) comment += sprintf(comment, "+%d", timezoneHours);

    }

    if (timezoneMinutes < 0) comment += sprintf(comment, ":%02d", -timezoneMinutes);

    if (timezoneMinutes > 0) comment += sprintf(comment, ":%02d", timezoneMinutes);

    if (memcmp(deploymentID, defaultDeploymentID, DEPLOYMENT_ID_LENGTH)) {

        comment += sprintf(comment, ") during deployment " SERIAL_NUMBER " ", FORMAT_SERIAL_NUMBER(deploymentID));

    } else {

        comment += sprintf(comment, ") by %s ", artist);

    }

    if (externalMicrophone) {

        comment += sprintf(comment, "using external microphone ");

    }

    static char *gainSettings[5] = {"low", "low-medium", "medium", "medium-high", "high"};

    comment += sprintf(comment, "at %s gain while battery was ", gainSettings[configSettings->gain]);

    if (extendedBatteryState == AM_EXT_BAT_LOW) {

        comment += sprintf(comment, "less than 2.5V");

    } else if (extendedBatteryState >= AM_EXT_BAT_FULL) {

        comment += sprintf(comment, "greater than 4.9V");

    } else {

        uint32_t batteryVoltage =  extendedBatteryState + AM_EXT_BAT_STATE_OFFSET / AM_BATTERY_STATE_INCREMENT;

        comment += sprintf(comment, "%01ld.%01ldV", batteryVoltage / 10, batteryVoltage % 10);

    }

    char *sign = temperature < 0 ? "-" : "";

    uint32_t temperatureInDecidegrees = ROUNDED_DIV(ABS(temperature), 100);

    comment += sprintf(comment, " and temperature was %s%ld.%ldC.", sign, temperatureInDecidegrees / 10, temperatureInDecidegrees % 10);
    
    bool amplitudeThresholdEnabled = configSettings->amplitudeThreshold > 0 || configSettings->enableAmplitudeThresholdDecibelScale || configSettings->enableAmplitudeThresholdPercentageScale;

    if (amplitudeThresholdEnabled) comment += sprintf(comment, " Amplitude threshold was ");

    if (configSettings->enableAmplitudeThresholdDecibelScale && configSettings->enableAmplitudeThresholdPercentageScale == false) {

        comment += formatDecibels(comment, configSettings->amplitudeThresholdDecibels);

    } else if (configSettings->enableAmplitudeThresholdPercentageScale && configSettings->enableAmplitudeThresholdDecibelScale == false) {

        comment += formatPercentage(comment, configSettings->amplitudeThresholdPercentageMantissa, configSettings->amplitudeThresholdPercentageExponent);

    } else if (amplitudeThresholdEnabled) {

        comment += sprintf(comment, "%d", configSettings->amplitudeThreshold);

    }

    if (amplitudeThresholdEnabled) comment += sprintf(comment, " with %ds minimum trigger duration.", configSettings->minimumTriggerDuration);

    uint16_t lowerFilterFreq = configSettings->lowerFilterFreq;

    uint16_t higherFilterFreq = configSettings->higherFilterFreq;

    if (filterType == LOW_PASS_FILTER) {

        comment += sprintf(comment, " Low-pass filter with frequency of %01d.%01dkHz applied.", higherFilterFreq / 10, higherFilterFreq % 10);

    } else if (filterType == BAND_PASS_FILTER) {

        comment += sprintf(comment, " Band-pass filter with frequencies of %01d.%01dkHz and %01d.%01dkHz applied.", lowerFilterFreq / 10, lowerFilterFreq % 10, higherFilterFreq / 10, higherFilterFreq % 10);

    } else if (filterType == HIGH_PASS_FILTER) {

        comment += sprintf(comment, " High-pass filter with frequency of %01d.%01dkHz applied.", lowerFilterFreq / 10, lowerFilterFreq % 10);

    }

    if (recordingState != RECORDING_OKAY) {

        comment += sprintf(comment, " Recording stopped due to ");

        if (recordingState == MICROPHONE_CHANGED) {

            comment += sprintf(comment, "microphone change.");

        } else if (recordingState == SWITCH_CHANGED) {

            comment += sprintf(comment, "switch position change.");

        } else if (recordingState == SUPPLY_VOLTAGE_LOW) {

            comment += sprintf(comment, "low voltage.");

        } else if (recordingState == FILE_SIZE_LIMITED) {

            comment += sprintf(comment, "file size limit.");

        }

    }

}

/* Function to write configuration to file */

static bool writeConfigurationToFile(configSettings_t *configSettings, uint8_t *firmwareDescription, uint8_t *firmwareVersion, uint8_t *serialNumber, uint8_t *deploymentID, uint8_t *defaultDeploymentID) {

    uint16_t length;

    static char configBuffer[512];

    RETURN_BOOL_ON_ERROR(AudioMoth_openFile("CONFIG.TXT"));

    length = sprintf(configBuffer, "Device ID                       : " SERIAL_NUMBER "\n", FORMAT_SERIAL_NUMBER(serialNumber));

    length += sprintf(configBuffer + length, "Firmware                        : %s (%d.%d.%d)\n\n", firmwareDescription, firmwareVersion[0], firmwareVersion[1], firmwareVersion[2]);

    if (memcmp(deploymentID, defaultDeploymentID, DEPLOYMENT_ID_LENGTH)) {

        length += sprintf(configBuffer + length, "Deployment ID                   : " SERIAL_NUMBER "\n\n", FORMAT_SERIAL_NUMBER(deploymentID));

    }

    length += sprintf(configBuffer + length, "Time zone                       : UTC");

    if (configSettings->timezoneHours < 0) {

        length += sprintf(configBuffer + length, "%d", configSettings->timezoneHours);

    } else if (configSettings->timezoneHours > 0) {

        length += sprintf(configBuffer + length, "+%d", configSettings->timezoneHours);

    } else {

        if (configSettings->timezoneMinutes < 0) length += sprintf(configBuffer + length, "-%d", configSettings->timezoneHours);

        if (configSettings->timezoneMinutes > 0) length += sprintf(configBuffer + length, "+%d", configSettings->timezoneHours);

    }

    if (configSettings->timezoneMinutes < 0) length += sprintf(configBuffer + length, ":%02d", -configSettings->timezoneMinutes);

    if (configSettings->timezoneMinutes > 0) length += sprintf(configBuffer + length, ":%02d", configSettings->timezoneMinutes);

    RETURN_BOOL_ON_ERROR(AudioMoth_writeToFile(configBuffer, length));

    length = sprintf(configBuffer, "\n\nSample rate (Hz)                : %ld\n", configSettings->sampleRate / configSettings->sampleRateDivider);

    static char *gainSettings[5] = {"Low", "Low-Medium", "Medium", "Medium-High", "High"};

    length += sprintf(configBuffer + length, "Gain                            : %s\n\n", gainSettings[configSettings->gain]);

    length += sprintf(configBuffer + length, "Sleep duration (s)              : ");

    if (configSettings->disableSleepRecordCycle) {

        length += sprintf(configBuffer + length, "-");

    } else {

        length += sprintf(configBuffer + length, "%d", configSettings->sleepDuration);

    }

    length += sprintf(configBuffer + length, "\nRecording duration (s)          : ");

    if (configSettings->disableSleepRecordCycle) {

        length += sprintf(configBuffer + length, "-");

    } else {

        length += sprintf(configBuffer + length, "%d", configSettings->recordDuration);

    }

    RETURN_BOOL_ON_ERROR(AudioMoth_writeToFile(configBuffer, length));

    length = sprintf(configBuffer, "\n\nActive recording periods        : %d\n", configSettings->activeStartStopPeriods);

    for (uint32_t i = 0; i < configSettings->activeStartStopPeriods; i += 1) {

        uint32_t startMinutes = configSettings->startStopPeriods[i].startMinutes;

        uint32_t stopMinutes = configSettings->startStopPeriods[i].stopMinutes;

        if (i == 0) length += sprintf(configBuffer + length, "\n");

        length += sprintf(configBuffer + length, "Recording period %ld              : %02ld:%02ld - %02ld:%02ld (UTC)\n", i + 1, startMinutes / 60, startMinutes % 60, stopMinutes / 60, stopMinutes % 60);

    }

    length += sprintf(configBuffer + length, "\nEarliest recording time         : ");

    if (configSettings->earliestRecordingTime == 0) {

        length += sprintf(configBuffer + length, "---------- --:--:--");

    } else {

        time_t rawTime = configSettings->earliestRecordingTime;

        struct tm *time = gmtime(&rawTime);

        length += sprintf(configBuffer + length, "%04d-%02d-%02d %02d:%02d:%02d (UTC)", 1900 + time->tm_year, time->tm_mon + 1, time->tm_mday, time->tm_hour, time->tm_min, time->tm_sec);

    }

    length += sprintf(configBuffer + length, "\nLatest recording time           : ");

    if (configSettings->latestRecordingTime == 0) {

        length += sprintf(configBuffer + length, "---------- --:--:--");

    } else {

        time_t rawTime = configSettings->latestRecordingTime;

        struct tm *time = gmtime(&rawTime);

        length += sprintf(configBuffer + length, "%04d-%02d-%02d %02d:%02d:%02d (UTC)", 1900 + time->tm_year, time->tm_mon + 1, time->tm_mday, time->tm_hour, time->tm_min, time->tm_sec);

    }

    RETURN_BOOL_ON_ERROR(AudioMoth_writeToFile(configBuffer, length));

    length = sprintf(configBuffer, "\n\nFilter                          : ");

    if (configSettings->lowerFilterFreq == 0 && configSettings->higherFilterFreq == 0) {

        length += sprintf(configBuffer + length, "-");

    } else if (configSettings->lowerFilterFreq == UINT16_MAX) {

        length += sprintf(configBuffer + length, "Low-pass (%d.%dkHz)", configSettings->higherFilterFreq / 10, configSettings->higherFilterFreq % 10);

    } else if (configSettings->higherFilterFreq == UINT16_MAX) {

        length += sprintf(configBuffer + length, "High-pass (%d.%dkHz)", configSettings->lowerFilterFreq / 10, configSettings->lowerFilterFreq % 10);

    } else {

        length += sprintf(configBuffer + length, "Band-pass (%d.%dkHz - %d.%dkHz)", configSettings->lowerFilterFreq / 10, configSettings->lowerFilterFreq % 10, configSettings->higherFilterFreq / 10, configSettings->higherFilterFreq % 10);

    }

    length += sprintf(configBuffer + length, "\nAmplitude threshold             : ");

    bool amplitudeThresholdEnabled = configSettings->amplitudeThreshold > 0 || configSettings->enableAmplitudeThresholdDecibelScale || configSettings->enableAmplitudeThresholdPercentageScale;

    if (configSettings->enableAmplitudeThresholdDecibelScale && configSettings->enableAmplitudeThresholdPercentageScale == false) {

        length += formatDecibels(configBuffer + length, configSettings->amplitudeThresholdDecibels);

    } else if (configSettings->enableAmplitudeThresholdPercentageScale && configSettings->enableAmplitudeThresholdDecibelScale == false) {

        length += formatPercentage(configBuffer + length, configSettings->amplitudeThresholdPercentageMantissa, configSettings->amplitudeThresholdPercentageExponent);

    } else if (amplitudeThresholdEnabled) {

        length += sprintf(configBuffer + length, "%d", configSettings->amplitudeThreshold);

    } else {

        length += sprintf(configBuffer + length, "-");

    }

    length += sprintf(configBuffer + length, "\nMinimum trigger duration (s)    : ");

    if (amplitudeThresholdEnabled) {

        length += sprintf(configBuffer + length, "%d", configSettings->minimumTriggerDuration);

    } else {

        length += sprintf(configBuffer + length, "-");

    }

    RETURN_BOOL_ON_ERROR(AudioMoth_writeToFile(configBuffer, length));

    length = sprintf(configBuffer, "\n\nEnable LED                      : %s\n", configSettings->enableLED ? "Yes" : "No");

    length += sprintf(configBuffer + length, "Enable low-voltage cut-off      : %s\n", configSettings->enableLowVoltageCutoff ? "Yes" : "No");

    length += sprintf(configBuffer + length, "Enable battery level indication : %s\n\n", configSettings->disableBatteryLevelDisplay ? "No" : configSettings->batteryLevelDisplayType == NIMH_LIPO_BATTERY_VOLTAGE ? "Yes (NiMH/LiPo voltage range)" : "Yes");

    length += sprintf(configBuffer + length, "Always require acoustic chime   : %s\n", configSettings->requireAcousticConfiguration ? "Yes" : "No");

    length += sprintf(configBuffer + length, "Disable 48Hz DC blocking filter : %s\n", configSettings->disable48HzDCBlockingFilter ? "Yes" : "No");

    length += sprintf(configBuffer + length, "Enable energy saver mode        : %s\n", configSettings->enableEnergySaverMode ? "Yes" : "No");

    RETURN_BOOL_ON_ERROR(AudioMoth_writeToFile(configBuffer, length));

    RETURN_BOOL_ON_ERROR(AudioMoth_closeFile());

    return true;

}

/* Backup domain variables */

static uint32_t *previousSwitchPosition = (uint32_t*)AM_BACKUP_DOMAIN_START_ADDRESS;

static uint32_t *timeOfNextRecording = (uint32_t*)(AM_BACKUP_DOMAIN_START_ADDRESS + 4);

static uint32_t *durationOfNextRecording = (uint32_t*)(AM_BACKUP_DOMAIN_START_ADDRESS + 8);

static uint32_t *writtenConfigurationToFile = (uint32_t*)(AM_BACKUP_DOMAIN_START_ADDRESS + 12);

static uint8_t *deploymentID = (uint8_t*)(AM_BACKUP_DOMAIN_START_ADDRESS + 16);

static uint32_t *readyToMakeRecordings = (uint32_t*)(AM_BACKUP_DOMAIN_START_ADDRESS + 24);

static uint32_t *recordingErrorHasOccurred = (uint32_t*)(AM_BACKUP_DOMAIN_START_ADDRESS + 28);

static uint32_t *recordingPreparationPeriod = (uint32_t*)(AM_BACKUP_DOMAIN_START_ADDRESS + 32);

static configSettings_t *configSettings = (configSettings_t*)(AM_BACKUP_DOMAIN_START_ADDRESS + 36);

/* Filter variables */

static AM_filterType_t requestedFilterType;

/* DMA transfer variable */

static uint32_t numberOfSamplesInDMATransfer;

/* SRAM buffer variables */

static volatile uint32_t writeBuffer;

static volatile uint32_t writeBufferIndex;

static int16_t* buffers[NUMBER_OF_BUFFERS];

/* Flag to start processing DMA transfers */

static volatile bool shouldProcessTransfers;

/* Compression buffers */

static bool writeIndicator[NUMBER_OF_BUFFERS];

static int16_t compressionBuffer[COMPRESSION_BUFFER_SIZE_IN_BYTES / NUMBER_OF_BYTES_IN_SAMPLE];

/* Audio configuration variables */

static bool audioConfigStateLED;

static bool audioConfigToggleLED;

static uint32_t audioConfigPulseCounter;

static bool acousticConfigurationPerformed;

static uint32_t secondsOfAcousticSignalStart;

static uint32_t millisecondsOfAcousticSignalStart;

/* Deployment ID variable */

static uint8_t defaultDeploymentID[DEPLOYMENT_ID_LENGTH];

/* Recording state */

static volatile bool microphoneChanged;

static volatile bool switchPositionChanged;

/* DMA buffers */

static int16_t primaryBuffer[MAXIMUM_SAMPLES_IN_DMA_TRANSFER];

static int16_t secondaryBuffer[MAXIMUM_SAMPLES_IN_DMA_TRANSFER];

/* Firmware version and description */

static uint8_t firmwareVersion[AM_FIRMWARE_VERSION_LENGTH] = {1, 6, 0};

static uint8_t firmwareDescription[AM_FIRMWARE_DESCRIPTION_LENGTH] = "AudioMoth-Firmware-Basic";

/* Function prototypes */

static void flashLedToIndicateBatteryLife(void);

static void scheduleRecording(uint32_t currentTime, uint32_t *timeOfNextRecording, uint32_t *durationOfNextRecording);

static AM_recordingState_t makeRecording(uint32_t timeOfNextRecording, uint32_t recordDuration, bool enableLED, AM_extendedBatteryState_t extendedBatteryState, int32_t temperature, uint32_t *fileOpenTime, uint32_t *fileOpenMilliseconds);

/* Functions of copy to and from the backup domain */

static void copyFromBackupDomain(uint8_t *dst, uint32_t *src, uint32_t length) {

    for (uint32_t i = 0; i < length; i += 1) {
        *(dst + i) = *((uint8_t*)src + i);
    }

}

static void copyToBackupDomain(uint32_t *dst, uint8_t *src, uint32_t length) {

    uint32_t value = 0;

    for (uint32_t i = 0; i < length / UINT32_SIZE_IN_BYTES; i += 1) {
        *(dst + i) = *((uint32_t*)src + i);
    }

    for (uint32_t i = 0; i < length % UINT32_SIZE_IN_BYTES; i += 1) {
        value = (value << BITS_PER_BYTE) + *(src + length - 1 - i);
    }

    *(dst + length / UINT32_SIZE_IN_BYTES) = value;

}

/* Function to select energy saver mode */

static bool isEnergySaverMode(configSettings_t *configSettings) {

    return configSettings->enableEnergySaverMode && configSettings->sampleRate / configSettings->sampleRateDivider <= ENERGY_SAVER_SAMPLE_RATE_THRESHOLD;

}

/* Main function */

int main(void) {

    /* Initialise device */

    AudioMoth_initialise();

    AM_switchPosition_t switchPosition = AudioMoth_getSwitchPosition();

    if (AudioMoth_isInitialPowerUp()) {

        *timeOfNextRecording = 0;

        *durationOfNextRecording = UINT32_MAX;

        *writtenConfigurationToFile = false;

        *previousSwitchPosition = AM_SWITCH_NONE;

        *readyToMakeRecordings = false;

        *recordingErrorHasOccurred = false;

        *recordingPreparationPeriod = INITIAL_PREPARATION_PERIOD;

        /* Copy default deployment ID */

        copyToBackupDomain((uint32_t*)deploymentID, (uint8_t*)defaultDeploymentID, DEPLOYMENT_ID_LENGTH);

        /* Check the persistent configuration */

        persistentConfigSettings_t *persistentConfigSettings = (persistentConfigSettings_t*)AM_FLASH_USER_DATA_ADDRESS;

        if (memcmp(persistentConfigSettings->firmwareVersion, firmwareVersion, AM_FIRMWARE_VERSION_LENGTH) == 0 && memcmp(persistentConfigSettings->firmwareDescription, firmwareDescription, AM_FIRMWARE_DESCRIPTION_LENGTH) == 0) {

            copyToBackupDomain((uint32_t*)configSettings, (uint8_t*)&persistentConfigSettings->configSettings, sizeof(configSettings_t));

        } else {

            copyToBackupDomain((uint32_t*)configSettings, (uint8_t*)&defaultConfigSettings, sizeof(configSettings_t));

        }

    }

    /* Handle the case that the switch is in USB position  */

    if (switchPosition == AM_SWITCH_USB) {

        if (configSettings->disableBatteryLevelDisplay == false && (*previousSwitchPosition == AM_SWITCH_DEFAULT || *previousSwitchPosition == AM_SWITCH_CUSTOM)) {

            flashLedToIndicateBatteryLife();

        }

        AudioMoth_handleUSB();

        SAVE_SWITCH_POSITION_AND_POWER_DOWN(DEFAULT_WAIT_INTERVAL);

    }

    /* Read the time */

    uint32_t currentTime;

    uint32_t currentMilliseconds;

    AudioMoth_getTime(&currentTime, &currentMilliseconds);

    /* Check if switch has just been moved to CUSTOM or DEFAULT */

    bool fileSystemEnabled = false;

    bool writtenConfigurationToFileInThisSession = false;

    if (switchPosition != *previousSwitchPosition) {

        /* Check there are active recording periods if the switch is in CUSTOM position */

        *readyToMakeRecordings = switchPosition == AM_SWITCH_DEFAULT || (switchPosition == AM_SWITCH_CUSTOM && configSettings->activeStartStopPeriods > 0);

        /* Check if acoustic configuration is required */

        if (*readyToMakeRecordings) {

            bool shouldPerformAcousticConfiguration = switchPosition == AM_SWITCH_CUSTOM && (AudioMoth_hasTimeBeenSet() == false || configSettings->requireAcousticConfiguration);

            bool listenForAcousticTone = shouldPerformAcousticConfiguration == false && switchPosition == AM_SWITCH_CUSTOM;

            if (listenForAcousticTone) {

                AudioConfig_enableAudioConfiguration();

                shouldPerformAcousticConfiguration = AudioConfig_listenForAudioConfigurationTone(AUDIO_CONFIG_TONE_TIMEOUT);

            }

            if (shouldPerformAcousticConfiguration) {

                AudioMoth_setRedLED(true);
                AudioMoth_setGreenLED(false);

                audioConfigPulseCounter = 0;

                audioConfigStateLED = false;

                audioConfigToggleLED = false;

                acousticConfigurationPerformed = false;

                if (listenForAcousticTone == false) {

                    AudioConfig_enableAudioConfiguration();

                }

                bool timedOut = AudioConfig_listenForAudioConfigurationPackets(listenForAcousticTone, AUDIO_CONFIG_PACKETS_TIMEOUT);

                AudioConfig_disableAudioConfiguration();

                if (acousticConfigurationPerformed) {

                    /* Ready to make a recording */

                    *readyToMakeRecordings = true;

                    /* Indicate success with LED flashes */

                    AudioMoth_setRedLED(false);
                    AudioMoth_setGreenLED(true);

                    AudioMoth_delay(1000);
                    AudioMoth_delay(1000);

                    AudioMoth_setGreenLED(false);

                    AudioMoth_delay(500);

                } else if (listenForAcousticTone && timedOut) {

                    /* Ready to make a recording */

                    *readyToMakeRecordings = true;

                    /* Turn off LED */

                    AudioMoth_setBothLED(false);

                } else {

                    /* Not ready to make a recording */

                    *readyToMakeRecordings = false;

                    /* Turn off LED */

                    AudioMoth_setBothLED(false);

                    /* Power down */

                    SAVE_SWITCH_POSITION_AND_POWER_DOWN(DEFAULT_WAIT_INTERVAL);

                }

            } else if (listenForAcousticTone) {

                AudioConfig_disableAudioConfiguration();

            }

        }

        /* Calculate time of next recording if ready to make a recording */

        if (*readyToMakeRecordings) {

            /* Enable energy saver mode */

            if (isEnergySaverMode(configSettings)) AudioMoth_setClockDivider(AM_HF_CLK_DIV2);

            /* Reset the error flag */

            *recordingErrorHasOccurred = false;

            /* Reset the recording preparation period to default */

            *recordingPreparationPeriod = INITIAL_PREPARATION_PERIOD;

            /* Reset persistent configuration write flag */

            *writtenConfigurationToFile = false;

            /* Try to write configuration to file */

            if (!fileSystemEnabled) fileSystemEnabled = AudioMoth_enableFileSystem();

            if (fileSystemEnabled) writtenConfigurationToFileInThisSession = writeConfigurationToFile(configSettings, firmwareDescription, firmwareVersion, (uint8_t*)AM_UNIQUE_ID_START_ADDRESS, deploymentID, defaultDeploymentID);

            /* Update the time and calculate earliest schedule start time */

            AudioMoth_getTime(&currentTime, &currentMilliseconds);

            uint32_t scheduleTime = currentTime + ROUNDED_UP_DIV(currentMilliseconds + *recordingPreparationPeriod, MILLISECONDS_IN_SECOND);

            /* Schedule the next recording */

            if (switchPosition == AM_SWITCH_CUSTOM) {

                scheduleRecording(scheduleTime, timeOfNextRecording, durationOfNextRecording);

            }

             /* Set parameters to start recording now */

            if (switchPosition == AM_SWITCH_DEFAULT) {

                *timeOfNextRecording = scheduleTime;

                *durationOfNextRecording = UINT32_MAX;

            }

        }

    }

    /* Flash warning if not ready to make recording */

    if (*readyToMakeRecordings == false) {

        FLASH_LED(Both, SHORT_LED_FLASH_DURATION)

        SAVE_SWITCH_POSITION_AND_POWER_DOWN(DEFAULT_WAIT_INTERVAL);

    }

    /* Make recording if appropriate */

    bool enableLED = (switchPosition == AM_SWITCH_DEFAULT) || configSettings->enableLED;

    int64_t timeUntilPreparationStart = (int64_t)*timeOfNextRecording * MILLISECONDS_IN_SECOND - (int64_t)*recordingPreparationPeriod - (int64_t)currentTime * MILLISECONDS_IN_SECOND - (int64_t)currentMilliseconds;

    if (timeUntilPreparationStart <= 0) {

        /* Enable energy saver mode */

        if (isEnergySaverMode(configSettings)) AudioMoth_setClockDivider(AM_HF_CLK_DIV2);

        /* Write configuration if not already done so */

        if (writtenConfigurationToFileInThisSession == false && *writtenConfigurationToFile == false) {

            if (!fileSystemEnabled) fileSystemEnabled = AudioMoth_enableFileSystem();

            if (fileSystemEnabled) {

                *writtenConfigurationToFile = writeConfigurationToFile(configSettings, firmwareDescription, firmwareVersion, (uint8_t*)AM_UNIQUE_ID_START_ADDRESS, deploymentID, defaultDeploymentID);

            }

        }

        /* Make the recording */

        uint32_t fileOpenTime;

        uint32_t fileOpenMilliseconds;

        AM_recordingState_t recordingState = RECORDING_OKAY;

        /* Measure battery voltage */

        uint32_t supplyVoltage = AudioMoth_getSupplyVoltage();

        AM_extendedBatteryState_t extendedBatteryState = AudioMoth_getExtendedBatteryState(supplyVoltage);

        /* Check if low voltage check is enabled and that the voltage is okay */

        bool okayToMakeRecording = true;

        if (configSettings->enableLowVoltageCutoff) {

            AudioMoth_enableSupplyMonitor();

            AudioMoth_setSupplyMonitorThreshold(MINIMUM_SUPPLY_VOLTAGE);

            okayToMakeRecording = AudioMoth_isSupplyAboveThreshold();

        }

        /* Make recording if okay */

        if (okayToMakeRecording) {

            AudioMoth_enableTemperature();

            int32_t temperature = AudioMoth_getTemperature();

            AudioMoth_disableTemperature();

            if (!fileSystemEnabled) fileSystemEnabled = AudioMoth_enableFileSystem();

            if (fileSystemEnabled)  {

                recordingState = makeRecording(*timeOfNextRecording, *durationOfNextRecording, enableLED, extendedBatteryState, temperature, &fileOpenTime, &fileOpenMilliseconds);

            } else {

                FLASH_LED(Both, LONG_LED_FLASH_DURATION);

                recordingState = SDCARD_WRITE_ERROR;

            }

        } else {

            if (enableLED) FLASH_LED(Both, LONG_LED_FLASH_DURATION);

            recordingState = SUPPLY_VOLTAGE_LOW;

        }

        /* Disable low voltage monitor if it was used */

        if (configSettings->enableLowVoltageCutoff) AudioMoth_disableSupplyMonitor();

        /* Enable the error warning flashes */

        if (switchPosition == AM_SWITCH_CUSTOM && (recordingState == SDCARD_WRITE_ERROR || recordingState == SUPPLY_VOLTAGE_LOW)) {

            *recordingErrorHasOccurred = true;

        }

        /* Update the preparation period */

        if (recordingState != SDCARD_WRITE_ERROR) {

            int64_t measuredPreparationPeriod = (int64_t)fileOpenTime * MILLISECONDS_IN_SECOND + (int64_t)fileOpenMilliseconds - (int64_t)currentTime * MILLISECONDS_IN_SECOND - (int64_t)currentMilliseconds;

            *recordingPreparationPeriod = MIN(MAXIMUM_PREPARATION_PERIOD, MAX(MINIMUM_PREPARATION_PERIOD, measuredPreparationPeriod + PREPARATION_PERIOD_INCREMENT));

        }

        /* Update the time and calculate earliest schedule start time */

        AudioMoth_getTime(&currentTime, &currentMilliseconds);

        uint32_t scheduleTime = currentTime + ROUNDED_UP_DIV(currentMilliseconds + *recordingPreparationPeriod, MILLISECONDS_IN_SECOND);

        /* Schedule the next recording */

        if (switchPosition == AM_SWITCH_CUSTOM) {

            if (recordingState == RECORDING_OKAY || recordingState == SUPPLY_VOLTAGE_LOW || recordingState == SDCARD_WRITE_ERROR) {

                /* Schedule as if the recording has ended correctly */

                scheduleTime = MAX(scheduleTime, *timeOfNextRecording + *durationOfNextRecording);

            }

            scheduleRecording(scheduleTime, timeOfNextRecording, durationOfNextRecording);

        }

        /* Set parameters to start recording now */

        if (switchPosition == AM_SWITCH_DEFAULT) {

            *timeOfNextRecording = scheduleTime;

            *durationOfNextRecording = UINT32_MAX;

        }

        /* Calculate the time until recording preparation should start */

        timeUntilPreparationStart = (int64_t)*timeOfNextRecording * MILLISECONDS_IN_SECOND - (int64_t)*recordingPreparationPeriod - (int64_t)currentTime * MILLISECONDS_IN_SECOND - (int64_t)currentMilliseconds;

    } else if (enableLED && switchPosition == AM_SWITCH_CUSTOM && timeUntilPreparationStart > MILLISECONDS_IN_SECOND) {

        /* Flash LED to indicate waiting */

        if (*recordingErrorHasOccurred) {

            FLASH_LED(Both, WAITING_LED_FLASH_DURATION);

        } else {

            FLASH_LED(Green, WAITING_LED_FLASH_DURATION);

        }

    }

    /* Determine how long to power down */

    uint32_t millisecondsToPowerDown = MAX(0, MIN(timeUntilPreparationStart - EM4_WAKEUP_PERIOD, WAITING_LED_FLASH_INTERVAL));

    SAVE_SWITCH_POSITION_AND_POWER_DOWN(millisecondsToPowerDown);

}

/* Time zone handler */

inline void AudioMoth_timezoneRequested(int8_t *timezoneHours, int8_t *timezoneMinutes) {

    *timezoneHours = configSettings->timezoneHours;

    *timezoneMinutes = configSettings->timezoneMinutes;

}


/* AudioMoth interrupt handlers */

inline void AudioMoth_handleSwitchInterrupt() {

    switchPositionChanged = true;

    AudioConfig_cancelAudioConfiguration();

}

inline void AudioMoth_handleMicrophoneChangeInterrupt() {

    microphoneChanged = true;

}

inline void AudioMoth_handleDirectMemoryAccessInterrupt(bool isPrimaryBuffer, int16_t **nextBuffer) {

    int16_t *source = secondaryBuffer;

    if (isPrimaryBuffer) source = primaryBuffer;

    /* Update the current buffer index and write buffer */

    bool thresholdExceeded = DigitalFilter_filter(source, buffers[writeBuffer] + writeBufferIndex, configSettings->sampleRateDivider, numberOfSamplesInDMATransfer, configSettings->amplitudeThreshold);

    if (shouldProcessTransfers) {

        writeIndicator[writeBuffer] |= thresholdExceeded;

        writeBufferIndex += numberOfSamplesInDMATransfer / configSettings->sampleRateDivider;

        if (writeBufferIndex == NUMBER_OF_SAMPLES_IN_BUFFER) {

            writeBufferIndex = 0;

            writeBuffer = (writeBuffer + 1) & (NUMBER_OF_BUFFERS - 1);

            writeIndicator[writeBuffer] = false;

        }

    }

}

/* AudioMoth USB message handlers */

inline void AudioMoth_usbFirmwareVersionRequested(uint8_t **firmwareVersionPtr) {

    *firmwareVersionPtr = firmwareVersion;

}

inline void AudioMoth_usbFirmwareDescriptionRequested(uint8_t **firmwareDescriptionPtr) {

    *firmwareDescriptionPtr = firmwareDescription;

}

inline void AudioMoth_usbApplicationPacketRequested(uint32_t messageType, uint8_t *transmitBuffer, uint32_t size) {

    /* Copy the current time to the USB packet */

    uint32_t currentTime;

    AudioMoth_getTime(&currentTime, NULL);

    memcpy(transmitBuffer + 1, &currentTime, UINT32_SIZE_IN_BYTES);

    /* Copy the unique ID to the USB packet */

    memcpy(transmitBuffer + 5, (uint8_t*)AM_UNIQUE_ID_START_ADDRESS, AM_UNIQUE_ID_SIZE_IN_BYTES);

    /* Copy the battery state to the USB packet */

    uint32_t supplyVoltage = AudioMoth_getSupplyVoltage();

    AM_batteryState_t batteryState = AudioMoth_getBatteryState(supplyVoltage);

    memcpy(transmitBuffer + 5 + AM_UNIQUE_ID_SIZE_IN_BYTES, &batteryState, 1);

    /* Copy the firmware version to the USB packet */

    memcpy(transmitBuffer + 6 + AM_UNIQUE_ID_SIZE_IN_BYTES, firmwareVersion, AM_FIRMWARE_VERSION_LENGTH);

    /* Copy the firmware description to the USB packet */

    memcpy(transmitBuffer + 6 + AM_UNIQUE_ID_SIZE_IN_BYTES + AM_FIRMWARE_VERSION_LENGTH, firmwareDescription, AM_FIRMWARE_DESCRIPTION_LENGTH);

}

inline void AudioMoth_usbApplicationPacketReceived(uint32_t messageType, uint8_t* receiveBuffer, uint8_t *transmitBuffer, uint32_t size) {

    /* Make persistent configuration settings data structure */

    static persistentConfigSettings_t persistentConfigSettings __attribute__ ((aligned(UINT32_SIZE_IN_BYTES)));

    memcpy(&persistentConfigSettings.firmwareVersion, &firmwareVersion, AM_FIRMWARE_VERSION_LENGTH);

    memcpy(&persistentConfigSettings.firmwareDescription, &firmwareDescription, AM_FIRMWARE_DESCRIPTION_LENGTH);

    memcpy(&persistentConfigSettings.configSettings, receiveBuffer + 1,  sizeof(configSettings_t));

    /* Implement energy saver mode changes */

    if (isEnergySaverMode(&persistentConfigSettings.configSettings)) {

        persistentConfigSettings.configSettings.sampleRate /= 2;
        persistentConfigSettings.configSettings.clockDivider /= 2;
        persistentConfigSettings.configSettings.sampleRateDivider /= 2;

    }

    /* Copy persistent configuration settings to flash */

    uint32_t numberOfBytes = ROUND_UP_TO_MULTIPLE(sizeof(persistentConfigSettings_t), UINT32_SIZE_IN_BYTES);

    bool success = AudioMoth_writeToFlashUserDataPage((uint8_t*)&persistentConfigSettings, numberOfBytes);

    if (success) {

        /* Copy the USB packet contents to the back-up register data structure location */

        copyToBackupDomain((uint32_t*)configSettings,  (uint8_t*)&persistentConfigSettings.configSettings, sizeof(configSettings_t));

        /* Copy the back-up register data structure to the USB packet */

        copyFromBackupDomain(transmitBuffer + 1, (uint32_t*)configSettings, sizeof(configSettings_t));

        /* Revert energy saver mode changes */

        configSettings_t *tempConfigSettings = (configSettings_t*)(transmitBuffer + 1);

        if (isEnergySaverMode(tempConfigSettings)) {

            tempConfigSettings->sampleRate *= 2;
            tempConfigSettings->clockDivider *= 2;
            tempConfigSettings->sampleRateDivider *= 2;

        }        

        /* Set the time */

        AudioMoth_setTime(configSettings->time, USB_CONFIG_TIME_CORRECTION);

    } else {

        /* Return blank configuration as error indicator */

        memset(transmitBuffer + 1, 0, sizeof(configSettings_t));

    }

}

/* Audio configuration handlers */

void AudioConfig_handleAudioConfigurationEvent(AC_audioConfigurationEvent_t event) {

    if (event == AC_EVENT_PULSE) {

        audioConfigPulseCounter = (audioConfigPulseCounter + 1) % AUDIO_CONFIG_PULSE_INTERVAL;

    } else if (event == AC_EVENT_START) {

        audioConfigStateLED = true;

        audioConfigToggleLED = true;

        AudioMoth_getTime(&secondsOfAcousticSignalStart, &millisecondsOfAcousticSignalStart);

    } else if (event == AC_EVENT_BYTE) {

        audioConfigToggleLED = !audioConfigToggleLED;

    } else if (event == AC_EVENT_BIT_ERROR || event == AC_EVENT_CRC_ERROR) {

        audioConfigStateLED = false;

    }

    AudioMoth_setGreenLED((audioConfigStateLED && audioConfigToggleLED) || (!audioConfigStateLED && !audioConfigPulseCounter));

}

void AudioConfig_handleAudioConfigurationPacket(uint8_t *receiveBuffer, uint32_t size) {

    bool isTimePacket = size == (UINT32_SIZE_IN_BYTES + UINT16_SIZE_IN_BYTES);

    bool isDeploymentPacket = size  == (UINT32_SIZE_IN_BYTES + UINT16_SIZE_IN_BYTES + DEPLOYMENT_ID_LENGTH);

    if (isTimePacket || isDeploymentPacket) {

        /* Copy time from the packet */

        uint32_t time;

        memcpy(&time, receiveBuffer, UINT32_SIZE_IN_BYTES);

        /* Calculate the time correction */

        uint32_t secondsOfAcousticSignalEnd;

        uint32_t millisecondsOfAcousticSignalEnd;

        AudioMoth_getTime(&secondsOfAcousticSignalEnd, &millisecondsOfAcousticSignalEnd);

        uint32_t millisecondTimeOffset = (secondsOfAcousticSignalEnd - secondsOfAcousticSignalStart) * MILLISECONDS_IN_SECOND + millisecondsOfAcousticSignalEnd - millisecondsOfAcousticSignalStart + AUDIO_CONFIG_TIME_CORRECTION;

        /* Set the time */

        AudioMoth_setTime(time + millisecondTimeOffset / MILLISECONDS_IN_SECOND, millisecondTimeOffset % MILLISECONDS_IN_SECOND);

        /* Set deployment */

        if (isDeploymentPacket) {

            copyToBackupDomain((uint32_t*)deploymentID, receiveBuffer + UINT32_SIZE_IN_BYTES + UINT16_SIZE_IN_BYTES, DEPLOYMENT_ID_LENGTH);

        }

        /* Indicate success */

        AudioConfig_cancelAudioConfiguration();

        acousticConfigurationPerformed = true;

    }

    /* Reset receive state */

    audioConfigStateLED = false;

}

/* Clear and encode the compression buffer */

static void clearCompressionBuffer() {

    for (uint32_t i = 0; i < COMPRESSION_BUFFER_SIZE_IN_BYTES / NUMBER_OF_BYTES_IN_SAMPLE; i += 1) {

        compressionBuffer[i] = 0;

    }

}

static void encodeCompressionBuffer(uint32_t numberOfCompressedBuffers) {

    for (uint32_t i = 0; i < UINT32_SIZE_IN_BITS; i += 1) {

        compressionBuffer[i] = numberOfCompressedBuffers & 0x01 ? 1 : -1;

        numberOfCompressedBuffers >>= 1;

    }

    for (uint32_t i = UINT32_SIZE_IN_BITS; i < COMPRESSION_BUFFER_SIZE_IN_BYTES / NUMBER_OF_BYTES_IN_SAMPLE; i += 1) {

        compressionBuffer[i] = 0;

    }

}

/* Generate filename from time */

static void generateFilename(char *fileName, uint32_t timestamp, bool amplitudeThresholdEnabled) {

    time_t rawTime = timestamp + configSettings->timezoneHours * SECONDS_IN_HOUR + configSettings->timezoneMinutes * SECONDS_IN_MINUTE;

    struct tm *time = gmtime(&rawTime);

    uint32_t length = sprintf(fileName, "%04d%02d%02d_%02d%02d%02d", 1900 + time->tm_year, time->tm_mon + 1, time->tm_mday, time->tm_hour, time->tm_min, time->tm_sec);

    char *extension = amplitudeThresholdEnabled ? "T.WAV" : ".WAV";

    strcpy(fileName + length, extension);

}

/* Save recording to SD card */

static AM_recordingState_t makeRecording(uint32_t timeOfNextRecording, uint32_t recordDuration, bool enableLED, AM_extendedBatteryState_t extendedBatteryState, int32_t temperature, uint32_t *fileOpenTime, uint32_t *fileOpenMilliseconds) {

    /* Initialise buffers */

    writeBuffer = 0;

    writeBufferIndex = 0;

    buffers[0] = (int16_t*)AM_EXTERNAL_SRAM_START_ADDRESS;

    for (uint32_t i = 1; i < NUMBER_OF_BUFFERS; i += 1) {
        buffers[i] = buffers[i - 1] + NUMBER_OF_SAMPLES_IN_BUFFER;
    }

    /* Calculate effective sample rate */

    uint32_t effectiveSampleRate = configSettings->sampleRate / configSettings->sampleRateDivider;

    /* Set up the digital filter */

    uint32_t blockingFilterFrequency = configSettings->disable48HzDCBlockingFilter ? LOW_DC_BLOCKING_FREQ : DEFAULT_DC_BLOCKING_FREQ;

    if (configSettings->lowerFilterFreq == 0 && configSettings->higherFilterFreq == 0) {

        requestedFilterType = NO_FILTER;

        DigitalFilter_designHighPassFilter(effectiveSampleRate, blockingFilterFrequency);

    } else if (configSettings->lowerFilterFreq == UINT16_MAX) {

        requestedFilterType = LOW_PASS_FILTER;

        DigitalFilter_designBandPassFilter(effectiveSampleRate, blockingFilterFrequency, FILTER_FREQ_MULTIPLIER * configSettings->higherFilterFreq);

    } else if (configSettings->higherFilterFreq == UINT16_MAX) {

        requestedFilterType = HIGH_PASS_FILTER;

        DigitalFilter_designHighPassFilter(effectiveSampleRate, MAX(blockingFilterFrequency, FILTER_FREQ_MULTIPLIER * configSettings->lowerFilterFreq));

    } else {

        requestedFilterType = BAND_PASS_FILTER;

        DigitalFilter_designBandPassFilter(effectiveSampleRate, MAX(blockingFilterFrequency, FILTER_FREQ_MULTIPLIER * configSettings->lowerFilterFreq), FILTER_FREQ_MULTIPLIER * configSettings->higherFilterFreq);

    }

    /* Calculate the sample multiplier */

    float sampleMultiplier = 16.0f / (float)(configSettings->oversampleRate * configSettings->sampleRateDivider);

    DigitalFilter_applyAdditionalGain(sampleMultiplier);

    /* Calculate the number of samples in each DMA transfer (while ensuring that number of samples written to the SRAM buffer on each DMA transfer is a power of two so each SRAM buffer is filled after an integer number of DMA transfers) */

    numberOfSamplesInDMATransfer = MAXIMUM_SAMPLES_IN_DMA_TRANSFER / configSettings->sampleRateDivider;

    while (numberOfSamplesInDMATransfer & (numberOfSamplesInDMATransfer - 1)) {

        numberOfSamplesInDMATransfer = numberOfSamplesInDMATransfer & (numberOfSamplesInDMATransfer - 1);

    }

    numberOfSamplesInDMATransfer *= configSettings->sampleRateDivider;

    /* Calculate the minimum amplitude threshold duration */

    uint32_t numberOfAmplitudeThresholdBuffers = ROUNDED_UP_DIV(configSettings->minimumTriggerDuration * effectiveSampleRate, NUMBER_OF_SAMPLES_IN_BUFFER);

    /* Initialise termination conditions */

    bool supplyVoltageLow = false;

    microphoneChanged = false;

    /* Initialise microphone for recording */

    AudioMoth_enableExternalSRAM();

    bool externalMicrophone = AudioMoth_enableMicrophone(configSettings->gain, configSettings->clockDivider, configSettings->acquisitionCycles, configSettings->oversampleRate);

    AudioMoth_initialiseDirectMemoryAccess(primaryBuffer, secondaryBuffer, numberOfSamplesInDMATransfer);

    AudioMoth_startMicrophoneSamples(configSettings->sampleRate);

    /* Show LED for SD card activity */

    if (enableLED) AudioMoth_setRedLED(true);

    /* Determine if amplitude threshold is enabled */

    bool amplitudeThresholdEnabled = configSettings->amplitudeThreshold > 0 || configSettings->enableAmplitudeThresholdDecibelScale || configSettings->enableAmplitudeThresholdPercentageScale;

    /* Open a file with the current local time as the name */

    static char filename[MAXIMUM_FILE_NAME_LENGTH];

    generateFilename(filename, timeOfNextRecording, amplitudeThresholdEnabled);

    FLASH_LED_AND_RETURN_ON_ERROR(AudioMoth_openFile(filename));

    AudioMoth_setRedLED(false);

    /* Measure the time difference from the start time */

    AudioMoth_getTime(fileOpenTime, fileOpenMilliseconds);

    /* Calculate time correction for sample rate */

    uint32_t numberOfSamplesInHeader = sizeof(wavHeader) / NUMBER_OF_BYTES_IN_SAMPLE;

    uint32_t sampleRateTimeOffset = ROUNDED_DIV(numberOfSamplesInHeader * MILLISECONDS_IN_SECOND, effectiveSampleRate);

    /* Calculate time until the recording should start */

    int64_t millisecondsUntilRecordingShouldStart = (int64_t)timeOfNextRecording * MILLISECONDS_IN_SECOND - (int64_t)*fileOpenTime * MILLISECONDS_IN_SECOND - (int64_t)*fileOpenMilliseconds - (int64_t)sampleRateTimeOffset;

    /* Calculate the actual recording start time if the intended start has been missed */

    uint32_t timeOffset = millisecondsUntilRecordingShouldStart < 0 ? 1 - millisecondsUntilRecordingShouldStart / MILLISECONDS_IN_SECOND : 0;

    recordDuration = timeOffset >= recordDuration ? 0 : recordDuration - timeOffset;

    /* Wait until the recording start time */

    uint32_t time;

    uint32_t milliseconds;

    while (millisecondsUntilRecordingShouldStart > 0 && !microphoneChanged && !switchPositionChanged) {

        AudioMoth_delay(1);

        AudioMoth_getTime(&time, &milliseconds);

        millisecondsUntilRecordingShouldStart = (int64_t)timeOfNextRecording * MILLISECONDS_IN_SECOND + (int64_t)timeOffset * MILLISECONDS_IN_SECOND - (int64_t)time * MILLISECONDS_IN_SECOND - (int64_t)milliseconds - (int64_t)sampleRateTimeOffset;

    }

    /* Calculate recording parameters */

    uint32_t maximumNumberOfSeconds = (MAXIMUM_WAV_FILE_SIZE - sizeof(wavHeader)) / NUMBER_OF_BYTES_IN_SAMPLE / effectiveSampleRate;

    bool fileSizeLimited = (recordDuration > maximumNumberOfSeconds);

    uint32_t numberOfSamples = effectiveSampleRate * (fileSizeLimited ? maximumNumberOfSeconds : recordDuration);

    /* Initialise main loop variables */

    uint32_t readBuffer = 0;

    uint32_t samplesWritten = 0;

    uint32_t buffersProcessed = 0;

    uint32_t numberOfCompressedBuffers = 0;

    uint32_t totalNumberOfCompressedSamples = 0;

    uint32_t amplitudeThresholdBuffersWritten = 0;

    bool amplitudeThresholdHasBeenTriggered = false;

    /* Start processing DMA transfers */

    shouldProcessTransfers = true;

    /* Main recording loop */

    while (samplesWritten < numberOfSamples + numberOfSamplesInHeader && !microphoneChanged && !switchPositionChanged && !supplyVoltageLow) {

        while (readBuffer != writeBuffer && samplesWritten < numberOfSamples + numberOfSamplesInHeader && !microphoneChanged && !switchPositionChanged && !supplyVoltageLow) {

            /* Determine the appropriate number of bytes to the SD card */

            uint32_t numberOfSamplesToWrite = MIN(numberOfSamples + numberOfSamplesInHeader - samplesWritten, NUMBER_OF_SAMPLES_IN_BUFFER);

            /* Check if this buffer should actually be written to the SD card */

            bool writeIndicated = writeIndicator[readBuffer] || amplitudeThresholdEnabled == false;

            amplitudeThresholdHasBeenTriggered |= writeIndicated;

            amplitudeThresholdBuffersWritten = writeIndicated ? 0 : amplitudeThresholdBuffersWritten + 1;

            bool shouldWriteThisSector = writeIndicated || (amplitudeThresholdHasBeenTriggered && amplitudeThresholdBuffersWritten < numberOfAmplitudeThresholdBuffers);

            /* Compress the buffer or write the buffer to SD card */

            if (shouldWriteThisSector == false && buffersProcessed > 0 && numberOfSamplesToWrite == NUMBER_OF_SAMPLES_IN_BUFFER) {

                numberOfCompressedBuffers += NUMBER_OF_BYTES_IN_SAMPLE * NUMBER_OF_SAMPLES_IN_BUFFER / COMPRESSION_BUFFER_SIZE_IN_BYTES;

            } else {

                /* Light LED during SD card write if appropriate */

                if (enableLED) AudioMoth_setRedLED(true);

                /* Encode and write compression buffer */

                if (numberOfCompressedBuffers > 0) {

                    encodeCompressionBuffer(numberOfCompressedBuffers);

                    totalNumberOfCompressedSamples += (numberOfCompressedBuffers - 1) * COMPRESSION_BUFFER_SIZE_IN_BYTES / NUMBER_OF_BYTES_IN_SAMPLE;

                    FLASH_LED_AND_RETURN_ON_ERROR(AudioMoth_writeToFile(compressionBuffer, COMPRESSION_BUFFER_SIZE_IN_BYTES));

                    numberOfCompressedBuffers = 0;

                }

                /* Either write the buffer or write a blank buffer */

                if (shouldWriteThisSector) {

                    FLASH_LED_AND_RETURN_ON_ERROR(AudioMoth_writeToFile(buffers[readBuffer], NUMBER_OF_BYTES_IN_SAMPLE * numberOfSamplesToWrite));

                } else {

                    clearCompressionBuffer();

                    uint32_t numberOfBlankSamplesToWrite = numberOfSamplesToWrite;

                    while (numberOfBlankSamplesToWrite > 0) {

                        uint32_t numberOfSmples = MIN(numberOfBlankSamplesToWrite, COMPRESSION_BUFFER_SIZE_IN_BYTES / NUMBER_OF_BYTES_IN_SAMPLE);

                        FLASH_LED_AND_RETURN_ON_ERROR(AudioMoth_writeToFile(compressionBuffer, NUMBER_OF_BYTES_IN_SAMPLE * numberOfSmples));

                        numberOfBlankSamplesToWrite -= numberOfSmples;

                    }

                }

                /* Clear LED */

                AudioMoth_setRedLED(false);

            }

            /* Increment buffer counters */

            readBuffer = (readBuffer + 1) & (NUMBER_OF_BUFFERS - 1);

            samplesWritten += numberOfSamplesToWrite;

            buffersProcessed += 1;

        }

        /* Check the voltage level */

        if (configSettings->enableLowVoltageCutoff && AudioMoth_isSupplyAboveThreshold() == false) {

            supplyVoltageLow = true;

        }

        /* Sleep until next DMA transfer is complete */

        AudioMoth_sleep();

    }

    /* Write the compression buffer files at the end */

    if (samplesWritten < numberOfSamples + numberOfSamplesInHeader && numberOfCompressedBuffers > 0) {

        /* Light LED during SD card write if appropriate */

        if (enableLED) AudioMoth_setRedLED(true);

        /* Encode and write compression buffer */

        encodeCompressionBuffer(numberOfCompressedBuffers);

        totalNumberOfCompressedSamples += (numberOfCompressedBuffers - 1) * COMPRESSION_BUFFER_SIZE_IN_BYTES / NUMBER_OF_BYTES_IN_SAMPLE;

        FLASH_LED_AND_RETURN_ON_ERROR(AudioMoth_writeToFile(compressionBuffer, COMPRESSION_BUFFER_SIZE_IN_BYTES));

        /* Clear LED */

        AudioMoth_setRedLED(false);

    }

    /* Determine recording state */

    AM_recordingState_t recordingState = microphoneChanged ? MICROPHONE_CHANGED :
                                         switchPositionChanged ? SWITCH_CHANGED :
                                         supplyVoltageLow ? SUPPLY_VOLTAGE_LOW :
                                         fileSizeLimited ? FILE_SIZE_LIMITED :
                                         RECORDING_OKAY;

    /* Initialise the WAV header */

    samplesWritten = MAX(numberOfSamplesInHeader, samplesWritten);

    setHeaderDetails(&wavHeader, effectiveSampleRate, samplesWritten - numberOfSamplesInHeader - totalNumberOfCompressedSamples);

    setHeaderComment(&wavHeader, configSettings, timeOfNextRecording + timeOffset, (uint8_t*)AM_UNIQUE_ID_START_ADDRESS, deploymentID, defaultDeploymentID, extendedBatteryState, temperature, externalMicrophone, recordingState, requestedFilterType);

    /* Write the header */

    if (enableLED) AudioMoth_setRedLED(true);

    FLASH_LED_AND_RETURN_ON_ERROR(AudioMoth_seekInFile(0));

    FLASH_LED_AND_RETURN_ON_ERROR(AudioMoth_writeToFile(&wavHeader, sizeof(wavHeader)));

    /* Close the file */

    FLASH_LED_AND_RETURN_ON_ERROR(AudioMoth_closeFile());

    AudioMoth_setRedLED(false);

    /* Rename the file if necessary */

    static char newFilename[MAXIMUM_FILE_NAME_LENGTH];

    if (timeOffset > 0) {

        generateFilename(newFilename, timeOfNextRecording + timeOffset, amplitudeThresholdEnabled);

        if (enableLED) AudioMoth_setRedLED(true);

        FLASH_LED_AND_RETURN_ON_ERROR(AudioMoth_renameFile(filename, newFilename));

        AudioMoth_setRedLED(false);

    }

    /* Return recording state */

    return recordingState;

}

/* Schedule recordings */

static void scheduleRecording(uint32_t currentTime, uint32_t *timeOfNextRecording, uint32_t *durationOfNextRecording) {

    /* Check number of active state stop periods */

    uint32_t activeStartStopPeriods = MIN(configSettings->activeStartStopPeriods, MAX_START_STOP_PERIODS);

    /* No active periods */

    if (activeStartStopPeriods == 0) {

        *timeOfNextRecording = UINT32_MAX;

        *durationOfNextRecording = 0;

        goto done;

    }

    /* Check if recording should be limited by earliest recording time */

    if (configSettings->earliestRecordingTime > 0) {

        currentTime = MAX(currentTime, configSettings->earliestRecordingTime);

    }

    /* Calculate the number of seconds of this day */

    time_t rawTime = currentTime;

    struct tm *time = gmtime(&rawTime);

    uint32_t currentSeconds = SECONDS_IN_HOUR * time->tm_hour + SECONDS_IN_MINUTE * time->tm_min + time->tm_sec;

    /* Check each active start stop period */

    for (uint32_t i = 0; i < activeStartStopPeriods; i += 1) {

        startStopPeriod_t *period = configSettings->startStopPeriods + i;

        /* Calculate the start and stop time of the current period */

        uint32_t startSeconds = SECONDS_IN_MINUTE * period->startMinutes;

        uint32_t stopSeconds = SECONDS_IN_MINUTE * period->stopMinutes;

        uint32_t durationOfStartStopPeriod = stopSeconds - startSeconds;

        /* Check if the start stop period has not yet started */

        if (currentSeconds <= startSeconds) {

            *timeOfNextRecording = currentTime + startSeconds - currentSeconds;

            if (configSettings->disableSleepRecordCycle) {

                *durationOfNextRecording = durationOfStartStopPeriod;

            } else {

                *durationOfNextRecording = MIN(configSettings->recordDuration, durationOfStartStopPeriod);

            }

            goto done;

        }

        /* Check if currently inside a start stop period */

        if (currentSeconds < stopSeconds) {

            /* Handle case with no sleep record cycle */

            uint32_t secondsFromStartOfPeriod = currentSeconds - startSeconds;

            if (configSettings->disableSleepRecordCycle) {

                *timeOfNextRecording = currentTime;

                *durationOfNextRecording = durationOfStartStopPeriod - secondsFromStartOfPeriod;;

                goto done;

            }

            /* Check if recording should start immediately */

            uint32_t durationOfCycle = configSettings->recordDuration + configSettings->sleepDuration;

            uint32_t partialCycle = secondsFromStartOfPeriod % durationOfCycle;

            if (partialCycle < configSettings->recordDuration) {

                *timeOfNextRecording = currentTime;

                *durationOfNextRecording = MIN(configSettings->recordDuration - partialCycle, durationOfStartStopPeriod - secondsFromStartOfPeriod);

                goto done;

            }

            /* Wait for next cycle to begin */

            secondsFromStartOfPeriod += durationOfCycle - partialCycle;

            if (secondsFromStartOfPeriod < durationOfStartStopPeriod) {

                *timeOfNextRecording = currentTime + durationOfCycle - partialCycle;

                *durationOfNextRecording = MIN(configSettings->recordDuration, durationOfStartStopPeriod - secondsFromStartOfPeriod);

                goto done;

            }

        }

    }

    /* Calculate time until first period tomorrow */

    startStopPeriod_t *firstPeriod = configSettings->startStopPeriods;

    uint32_t startSeconds = SECONDS_IN_MINUTE * firstPeriod->startMinutes;

    uint32_t stopSeconds = SECONDS_IN_MINUTE * firstPeriod->stopMinutes;

    uint32_t durationOfStartStopPeriod = stopSeconds - startSeconds;

    *timeOfNextRecording = currentTime + (SECONDS_IN_DAY - currentSeconds) + startSeconds;

    if (configSettings->disableSleepRecordCycle) {

        *durationOfNextRecording = durationOfStartStopPeriod;

    } else {

        *durationOfNextRecording = MIN(configSettings->recordDuration, durationOfStartStopPeriod);

    }

done:

    /* Check if recording should be limited by last recording time */

    if (configSettings->latestRecordingTime > 0) {

        if (*timeOfNextRecording >= configSettings->latestRecordingTime) {

            *timeOfNextRecording = UINT32_MAX;

            *durationOfNextRecording = 0;

        } else {

            int64_t excessTime = (int64_t)*timeOfNextRecording + (int64_t)*durationOfNextRecording - (int64_t)configSettings->latestRecordingTime;

            if (excessTime > 0) {

                *durationOfNextRecording -= excessTime;

            }

        }

    }

}

/* Flash LED according to battery life */

static void flashLedToIndicateBatteryLife(void) {

    uint32_t numberOfFlashes = LOW_BATTERY_LED_FLASHES;

    uint32_t supplyVoltage = AudioMoth_getSupplyVoltage();

    if (configSettings->batteryLevelDisplayType == NIMH_LIPO_BATTERY_VOLTAGE) {

        /* Set number of flashes according to battery voltage */

        AM_extendedBatteryState_t batteryState = AudioMoth_getExtendedBatteryState(supplyVoltage);

        if (batteryState > AM_EXT_BAT_4V3) {

            numberOfFlashes = 1;

        } else if (batteryState > AM_EXT_BAT_3V5) {

            numberOfFlashes = AM_EXT_BAT_4V4 - batteryState;

        }

    } else {

        /* Set number of flashes according to battery state */

        AM_batteryState_t batteryState = AudioMoth_getBatteryState(supplyVoltage);

        if (batteryState > AM_BATTERY_LOW) {

            numberOfFlashes = (batteryState >= AM_BATTERY_4V6) ? 4 : (batteryState >= AM_BATTERY_4V4) ? 3 : (batteryState >= AM_BATTERY_4V0) ? 2 : 1;

        }

    }

    /* Flash LED */

    for (uint32_t i = 0; i < numberOfFlashes; i += 1) {

        FLASH_LED(Red, SHORT_LED_FLASH_DURATION)

        if (numberOfFlashes == LOW_BATTERY_LED_FLASHES) {

            AudioMoth_delay(SHORT_LED_FLASH_DURATION);

        } else {

            AudioMoth_delay(LONG_LED_FLASH_DURATION);

        }

    }

}
