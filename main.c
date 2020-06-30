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
#include "digitalFilter.h"

/* Useful time constants */

#define SECONDS_IN_MINUTE                   60
#define SECONDS_IN_HOUR                     (60 * SECONDS_IN_MINUTE)
#define SECONDS_IN_DAY                      (24 * SECONDS_IN_HOUR)

/* Useful type constants */

#define BITS_PER_BYTE                       8
#define UINT32_SIZE_IN_BITS                 32
#define UINT32_SIZE_IN_BYTES                4

/* Sleep and LED constants */

#define DEFAULT_WAIT_INTERVAL               1

#define WAITING_LED_FLASH_INTERVAL          2
#define WAITING_LED_FLASH_DURATION          10

#define LOW_BATTERY_LED_FLASHES             10

#define SHORT_LED_FLASH_DURATION            100
#define LONG_LED_FLASH_DURATION             500

/* SRAM buffer constants */

#define NUMBER_OF_BUFFERS                   8
#define NUMBER_OF_BYTES_IN_SAMPLE           2
#define EXTERNAL_SRAM_SIZE_IN_SAMPLES       (AM_EXTERNAL_SRAM_SIZE_IN_BYTES / NUMBER_OF_BYTES_IN_SAMPLE)
#define NUMBER_OF_SAMPLES_IN_BUFFER         (EXTERNAL_SRAM_SIZE_IN_SAMPLES / NUMBER_OF_BUFFERS)
#define NUMBER_OF_SAMPLES_IN_DMA_TRANSFER   1024

/* Microphone warm-up constant */

#define FRACTION_OF_SECOND_FOR_WARMUP       2

/* Compression constants */

#define COMPRESSION_BUFFER_SIZE_IN_BYTES    512

/* File size constants */

#define MAXIMUM_WAV_FILE_SIZE              (UINT32_MAX - 1)

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

#define DC_BLOCKING_FREQ                    48

/* Supply monitor constant */

#define MINIMUM_SUPPLY_VOLTAGE              2800

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

#define SAVE_SWITCH_POSITION_AND_POWER_DOWN(duration) { \
    *previousSwitchPosition = switchPosition; \
    AudioMoth_powerDownAndWake(duration, true); \
}

#define ABS(a)                                  ((a) < (0) ? (-a) : (a))

#define MIN(a, b)                               ((a) < (b) ? (a) : (b))

#define MAX(a, b)                               ((a) > (b) ? (a) : (b))

#define ROUNDED_DIV(a, b)                       (((a) + (b/2)) / (b))

/* Recording state enumeration */

typedef enum {RECORDING_OKAY, FILE_SIZE_LIMITED, SUPPLY_VOLTAGE_LOW, SWITCH_CHANGED, SDCARD_WRITE_ERROR} AM_recordingState_t;

/* Filter type enumeration */

typedef enum {NO_FILTER, LOW_PASS_FILTER, BAND_PASS_FILTER, HIGH_PASS_FILTER} AM_filterType_t;

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

/* Functions to set WAV header details and comment */

static void setHeaderDetails(wavHeader_t *wavHeader, uint32_t sampleRate, uint32_t numberOfSamples) {

    wavHeader->wavFormat.samplesPerSecond = sampleRate;
    wavHeader->wavFormat.bytesPerSecond = NUMBER_OF_BYTES_IN_SAMPLE * sampleRate;
    wavHeader->data.size = NUMBER_OF_BYTES_IN_SAMPLE * numberOfSamples;
    wavHeader->riff.size = NUMBER_OF_BYTES_IN_SAMPLE * numberOfSamples + sizeof(wavHeader_t) - sizeof(chunk_t);

}

static void setHeaderComment(wavHeader_t *wavHeader, uint32_t currentTime, int8_t timezoneHours, int8_t timezoneMinutes, uint8_t *serialNumber, uint32_t gain, AM_extendedBatteryState_t extendedBatteryState, int32_t temperature, bool switchPositionChanged, bool supplyVoltageLow, bool fileSizeLimited, uint32_t amplitudeThreshold, AM_filterType_t filterType, uint32_t lowerFilterFreq, uint32_t higherFilterFreq) {

    time_t rawtime = currentTime + timezoneHours * SECONDS_IN_HOUR + timezoneMinutes * SECONDS_IN_MINUTE;

    struct tm *time = gmtime(&rawtime);

    /* Format artist field */

    char *artist = wavHeader->iart.artist;

    sprintf(artist, "AudioMoth %08X%08X", (unsigned int)*((uint32_t*)serialNumber + 1), (unsigned int)*((uint32_t*)serialNumber));

    /* Format comment field */

    char *comment = wavHeader->icmt.comment;

    comment += sprintf(comment, "Recorded at %02d:%02d:%02d %02d/%02d/%04d (UTC", time->tm_hour, time->tm_min, time->tm_sec, time->tm_mday, 1 + time->tm_mon, 1900 + time->tm_year);

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

    static char *gainSettings[5] = {"low", "low-medium", "medium", "medium-high", "high"};

    comment +=  sprintf(comment, ") by %s at %s gain setting while battery state was ", artist, gainSettings[gain]);

    if (extendedBatteryState == AM_EXT_BAT_LOW) {

        comment += sprintf(comment, "less than 2.5V");

    } else if (extendedBatteryState >= AM_EXT_BAT_FULL) {

        comment += sprintf(comment, "greater than 4.9V");

    } else {

        uint32_t batteryVoltage =  extendedBatteryState + AM_EXT_BAT_STATE_OFFSET / AM_BATTERY_STATE_INCREMENT;

        comment += sprintf(comment, "%01d.%01dV", (unsigned int)batteryVoltage / 10, (unsigned int)batteryVoltage % 10);

    }

    char *sign = temperature < 0 ? "-" : "";

    uint32_t temperatureInDecidegrees = ROUNDED_DIV(ABS(temperature), 100);

    comment += sprintf(comment, " and temperature was %s%d.%dC.", sign, (unsigned int)temperatureInDecidegrees / 10, (unsigned int)temperatureInDecidegrees % 10);

    if (amplitudeThreshold > 0) {

        comment += sprintf(comment, " Amplitude threshold was %d.", (unsigned int)amplitudeThreshold);

    }

    if (filterType == LOW_PASS_FILTER) {

        comment += sprintf(comment, " Low-pass filter applied with cut-off frequency of %01d.%01dkHz.", (unsigned int)higherFilterFreq / 10, (unsigned int)higherFilterFreq % 10);

    } else if (filterType == BAND_PASS_FILTER) {

        comment += sprintf(comment, " Band-pass filter applied with cut-off frequencies of %01d.%01dkHz and %01d.%01dkHz.", (unsigned int)lowerFilterFreq / 10, (unsigned int)lowerFilterFreq % 10, (unsigned int)higherFilterFreq / 10, (unsigned int)higherFilterFreq % 10);

    } else if (filterType == HIGH_PASS_FILTER) {

        comment += sprintf(comment, " High-pass filter applied with cut-off frequency of %01d.%01dkHz.", (unsigned int)lowerFilterFreq / 10, (unsigned int)lowerFilterFreq % 10);

    }

    if (supplyVoltageLow || switchPositionChanged || fileSizeLimited) {

        comment += sprintf(comment, " Recording cancelled before completion due to ");

        if (switchPositionChanged) {

            comment += sprintf(comment, "change of switch position.");

        } else if (supplyVoltageLow) {

            comment += sprintf(comment, "low voltage.");

        } else if (fileSizeLimited) {

            comment += sprintf(comment, "file size limit.");

        }

    }

}

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
    .activeStartStopPeriods = 0,
    .startStopPeriods = {
        {.startMinutes = 000, .stopMinutes = 060},
        {.startMinutes = 120, .stopMinutes = 180},
        {.startMinutes = 240, .stopMinutes = 300},
        {.startMinutes = 360, .stopMinutes = 420},
        {.startMinutes = 480, .stopMinutes = 540}
    },
    .timezoneHours = 0,
    .enableLowVoltageCutoff = 0,
    .disableBatteryLevelDisplay = 0,
    .timezoneMinutes = 0,
    .disableSleepRecordCycle = 0,
    .earliestRecordingTime = 0,
    .latestRecordingTime = 0,
    .lowerFilterFreq = 0,
    .higherFilterFreq = 0,
    .amplitudeThreshold = 0
};

/* Function to write configuration to file */

static bool writeConfigurationToFile(configSettings_t *configSettings, uint8_t *firmwareDescription, uint8_t *firmwareVersion, uint8_t *serialNumber) {

    uint16_t length;

    static char configBuffer[512];

    RETURN_BOOL_ON_ERROR(AudioMoth_openFile("CONFIG.TXT"));

    length = sprintf(configBuffer, "\nDevice ID                       : %08X%08X\n", (unsigned int)*((uint32_t*)serialNumber + 1), (unsigned int)*((uint32_t*)serialNumber));

    length += sprintf(configBuffer + length, "Firmware                        : %s (%d.%d.%d)\n\n", firmwareDescription, firmwareVersion[0], firmwareVersion[1], firmwareVersion[2]);

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

    length = sprintf(configBuffer, "\n\nSample rate (Hz)                : %d\n", (unsigned int)configSettings->sampleRate / (unsigned int)configSettings->sampleRateDivider);

    static char *gainSettings[5] = {"Low", "Low-Medium", "Medium", "Medium-High", "High"};

    length += sprintf(configBuffer + length, "Gain                            : %s\n\n", gainSettings[configSettings->gain]);

    length += sprintf(configBuffer + length, "Sleep duration (s)              : ");

    if (configSettings->disableSleepRecordCycle) {

        length += sprintf(configBuffer + length, "-");

    } else {

        length += sprintf(configBuffer + length, "%d", (unsigned int)configSettings->sleepDuration);

    }

    length += sprintf(configBuffer + length, "\nRecording duration (s)          : ");

    if (configSettings->disableSleepRecordCycle) {

        length += sprintf(configBuffer + length, "-");

    } else {

        length += sprintf(configBuffer + length, "%d", (unsigned int)configSettings->recordDuration);

    }

    RETURN_BOOL_ON_ERROR(AudioMoth_writeToFile(configBuffer, length));

    length = sprintf(configBuffer, "\n\nActive recording periods        : %d\n", (unsigned int)configSettings->activeStartStopPeriods);

    for (uint32_t i = 0; i < configSettings->activeStartStopPeriods; i += 1) {

        uint32_t startMinutes = configSettings->startStopPeriods[i].startMinutes;

        uint32_t stopMinutes = configSettings->startStopPeriods[i].stopMinutes;

        if (i == 0) length += sprintf(configBuffer + length, "\n");

        length += sprintf(configBuffer + length, "Recording period %d              : %02d:%02d - %02d:%02d (UTC)\n", (unsigned int)i + 1, (unsigned int)startMinutes / 60, (unsigned int)startMinutes % 60, (unsigned int)stopMinutes / 60, (unsigned int)stopMinutes % 60);

    }

    length += sprintf(configBuffer + length, "\nEarliest recording time         : ");

    if (configSettings->earliestRecordingTime == 0) {

        length += sprintf(configBuffer + length, "---------- --:--:--");

    } else {

        struct tm *time = gmtime((time_t*)&configSettings->earliestRecordingTime);

        length += sprintf(configBuffer + length, "%04d-%02d-%02d %02d:%02d:%02d (UTC)", 1900 + time->tm_year, time->tm_mon + 1, time->tm_mday, time->tm_hour, time->tm_min, time->tm_sec);

    }

    length += sprintf(configBuffer + length, "\nLatest recording time           : ");

    if (configSettings->latestRecordingTime == 0) {

        length += sprintf(configBuffer + length, "---------- --:--:--");

    } else {

        struct tm *time = gmtime((time_t*)&configSettings->latestRecordingTime);

        length += sprintf(configBuffer + length, "%04d-%02d-%02d %02d:%02d:%02d (UTC)", 1900 + time->tm_year, time->tm_mon + 1, time->tm_mday, time->tm_hour, time->tm_min, time->tm_sec);

    }

    RETURN_BOOL_ON_ERROR(AudioMoth_writeToFile(configBuffer, length));

    length = sprintf(configBuffer, "\n\nFilter                          : ");

    if (configSettings->lowerFilterFreq == 0 && configSettings->higherFilterFreq == 0) {

        length += sprintf(configBuffer + length, "-");

    } else if (configSettings->lowerFilterFreq == UINT16_MAX) {

        length += sprintf(configBuffer + length, "Low-pass (%d.%dkHz)", (unsigned int)configSettings->higherFilterFreq / 10, (unsigned int)configSettings->higherFilterFreq % 10);

    } else if (configSettings->higherFilterFreq == UINT16_MAX) {

        length += sprintf(configBuffer + length, "High-pass (%d.%dkHz)", (unsigned int)configSettings->lowerFilterFreq / 10, (unsigned int)configSettings->lowerFilterFreq % 10);

    } else {

        length += sprintf(configBuffer + length, "Band-pass (%d.%dkHz - %d.%dkHz)", (unsigned int)configSettings->lowerFilterFreq / 10, (unsigned int)configSettings->lowerFilterFreq % 10, (unsigned int)configSettings->higherFilterFreq / 10, (unsigned int)configSettings->higherFilterFreq % 10);

    }

    length += sprintf(configBuffer + length, "\nAmplitude threshold             : ");

    if (configSettings->amplitudeThreshold == 0) {

        length += sprintf(configBuffer + length, "-");

    } else {

        length += sprintf(configBuffer + length, "%d", (unsigned int)configSettings->amplitudeThreshold);

    }

    RETURN_BOOL_ON_ERROR(AudioMoth_writeToFile(configBuffer, length));

    length = sprintf(configBuffer, "\n\nEnable LED                      : %s\n", configSettings->enableLED ? "true" : "false");

    length += sprintf(configBuffer + length, "Enable low-voltage cutoff       : %s\n", configSettings->enableLowVoltageCutoff ? "true" : "false");

    length += sprintf(configBuffer + length, "Enable battery level indication : %s\n", configSettings->disableBatteryLevelDisplay ? "false" : "true");

    RETURN_BOOL_ON_ERROR(AudioMoth_writeToFile(configBuffer, length));

    RETURN_BOOL_ON_ERROR(AudioMoth_closeFile());

    return true;

}

/* Backup domain variables */

static uint32_t *previousSwitchPosition = (uint32_t*)AM_BACKUP_DOMAIN_START_ADDRESS;

static uint32_t *timeOfNextRecording = (uint32_t*)(AM_BACKUP_DOMAIN_START_ADDRESS + 4);

static uint32_t *durationOfNextRecording = (uint32_t*)(AM_BACKUP_DOMAIN_START_ADDRESS + 8);

static uint32_t *writtenConfigurationToFile = (uint32_t*)(AM_BACKUP_DOMAIN_START_ADDRESS + 12);

static configSettings_t *configSettings = (configSettings_t*)(AM_BACKUP_DOMAIN_START_ADDRESS + 16);

/* Filter variables */

static AM_filterType_t requestedFilterType;

/* SRAM buffer variables */

static volatile uint32_t writeBuffer;

static volatile uint32_t writeBufferIndex;

static int16_t* buffers[NUMBER_OF_BUFFERS];

/* Initial microphone warm-up period settings */

static uint32_t dmaTransfersToSkip;

static volatile uint32_t dmaTransfersProcessed;

/* Compression buffers */

static bool writeIndicator[NUMBER_OF_BUFFERS];

static int16_t compressionBuffer[COMPRESSION_BUFFER_SIZE_IN_BYTES / NUMBER_OF_BYTES_IN_SAMPLE];

/* Recording state */

static volatile bool switchPositionChanged;

/* DMA buffers */

static int16_t primaryBuffer[NUMBER_OF_SAMPLES_IN_DMA_TRANSFER];

static int16_t secondaryBuffer[NUMBER_OF_SAMPLES_IN_DMA_TRANSFER];

/* Current recording file name */

static char fileName[32];

/* Firmware version and description */

static uint8_t firmwareVersion[AM_FIRMWARE_VERSION_LENGTH] = {1, 4, 2};

static uint8_t firmwareDescription[AM_FIRMWARE_DESCRIPTION_LENGTH] = "AudioMoth-Firmware-Basic";

/* Function prototypes */

static void flashLedToIndicateBatteryLife(void);

static void scheduleRecording(uint32_t currentTime, uint32_t *timeOfNextRecording, uint32_t *durationOfNextRecording);

static AM_recordingState_t makeRecording(uint32_t currentTime, uint32_t recordDuration, bool enableLED, AM_extendedBatteryState_t extendedBatteryState, int32_t temperature);

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

/* Main function */

int main(void) {

    /* Initialise device */

    AudioMoth_initialise();

    AM_switchPosition_t switchPosition = AudioMoth_getSwitchPosition();

    if (AudioMoth_isInitialPowerUp()) {

        *timeOfNextRecording = 0;

        *durationOfNextRecording = 0;

        *writtenConfigurationToFile = false;

        *previousSwitchPosition = AM_SWITCH_NONE;

        copyToBackupDomain((uint32_t*)configSettings, (uint8_t*)&defaultConfigSettings, sizeof(configSettings_t));

    } else {

        /* Indicate battery state is not initial power up and switch has been moved into USB if enabled */

        if (switchPosition != *previousSwitchPosition && switchPosition == AM_SWITCH_USB && !configSettings->disableBatteryLevelDisplay) {

            flashLedToIndicateBatteryLife();

        }

    }

    /* Handle the case that the switch is in USB position  */

    if (switchPosition == AM_SWITCH_USB) {

        AudioMoth_handleUSB();

        SAVE_SWITCH_POSITION_AND_POWER_DOWN(DEFAULT_WAIT_INTERVAL);

    }

    /* Handle the case that the switch is in CUSTOM position but the time has not been set or their are no active recording periods */

    if (switchPosition == AM_SWITCH_CUSTOM && (AudioMoth_hasTimeBeenSet() == false || configSettings->activeStartStopPeriods == 0)) {

        FLASH_LED(Both, SHORT_LED_FLASH_DURATION)

        SAVE_SWITCH_POSITION_AND_POWER_DOWN(DEFAULT_WAIT_INTERVAL);

    }

    /* Calculate time of next recording if switch has changed position */

    uint32_t currentTime;

    bool fileSystemEnabled = false;

    bool writtenConfigurationToFileInThisSession = false;

    AudioMoth_getTime(&currentTime, NULL);

    if (switchPosition != *previousSwitchPosition) {

         /* Reset persistent configuration write flag */

         *writtenConfigurationToFile = false;

         /* Try to write configuration to file */

         if (!fileSystemEnabled) fileSystemEnabled = AudioMoth_enableFileSystem();

         if (fileSystemEnabled) writtenConfigurationToFileInThisSession = writeConfigurationToFile(configSettings, firmwareDescription, firmwareVersion, (uint8_t*)AM_UNIQUE_ID_START_ADDRESS);

         /* Schedule the next recording */

         if (switchPosition == AM_SWITCH_CUSTOM) {

             scheduleRecording(currentTime, timeOfNextRecording, durationOfNextRecording);

         }

         /* Set parameters to start recording now */

         if (switchPosition == AM_SWITCH_DEFAULT) {

             *timeOfNextRecording = currentTime;

             *durationOfNextRecording = UINT32_MAX;

         }

    }

    /* Make recording if appropriate */

    bool enableLED = (switchPosition == AM_SWITCH_DEFAULT) || configSettings->enableLED;

    if (currentTime >= *timeOfNextRecording) {

        /* Write configuration if not already done so */

        if (!writtenConfigurationToFileInThisSession && *writtenConfigurationToFile == false) {

            if (!fileSystemEnabled) fileSystemEnabled = AudioMoth_enableFileSystem();

            if (fileSystemEnabled) *writtenConfigurationToFile = writeConfigurationToFile(configSettings, firmwareDescription, firmwareVersion, (uint8_t*)AM_UNIQUE_ID_START_ADDRESS);

        }

        /* Reduce the recording duration if necessary */

        if (switchPosition == AM_SWITCH_CUSTOM) {

            uint32_t missedSeconds = MIN(currentTime - *timeOfNextRecording, *durationOfNextRecording);

            *durationOfNextRecording -= missedSeconds;

        }

        /* Make the recording */

        AM_recordingState_t recordingState = RECORDING_OKAY;

        if (*durationOfNextRecording > 0) {

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

                    recordingState = makeRecording(currentTime, *durationOfNextRecording, enableLED, extendedBatteryState, temperature);

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

        }

        /* Schedule next recording if recording did not end early due to the file size limit (otherwise restart immediately) */

        if (switchPosition == AM_SWITCH_CUSTOM && recordingState != FILE_SIZE_LIMITED) {

            scheduleRecording(currentTime + *durationOfNextRecording, timeOfNextRecording, durationOfNextRecording);

        }

        /* Use longer power-down period if an error has occurred (otherwise restart immediately) */

        if (switchPosition == AM_SWITCH_DEFAULT && (recordingState == SDCARD_WRITE_ERROR || recordingState == SUPPLY_VOLTAGE_LOW)) {

            SAVE_SWITCH_POSITION_AND_POWER_DOWN(DEFAULT_WAIT_INTERVAL);

        }

    } else if (enableLED) {

        /* Flash LED to indicate waiting */

        FLASH_LED(Green, WAITING_LED_FLASH_DURATION);

    }

    /* Determine how long to power down */

    uint32_t secondsToSleep = 0;

    if (*timeOfNextRecording > currentTime) {

        secondsToSleep = MIN(*timeOfNextRecording - currentTime, WAITING_LED_FLASH_INTERVAL);

    }

    /* Power down */

    SAVE_SWITCH_POSITION_AND_POWER_DOWN(secondsToSleep);

}

/* Time zone handler */

inline void AudioMoth_timezoneRequested(int8_t *timezoneHours, int8_t *timezoneMinutes) {

    *timezoneHours = configSettings->timezoneHours;

    *timezoneMinutes = configSettings->timezoneMinutes;

}


/* AudioMoth interrupt handlers */

inline void AudioMoth_handleSwitchInterrupt() {

    switchPositionChanged = true;

}

inline void AudioMoth_handleMicrophoneInterrupt(int16_t sample) { }

inline void AudioMoth_handleDirectMemoryAccessInterrupt(bool isPrimaryBuffer, int16_t **nextBuffer) {

    int16_t *source = secondaryBuffer;

    if (isPrimaryBuffer) source = primaryBuffer;

    /* Update the current buffer index and write buffer */

    bool thresholdExceeded = DigitalFilter_filter(source, buffers[writeBuffer] + writeBufferIndex, configSettings->sampleRateDivider, NUMBER_OF_SAMPLES_IN_DMA_TRANSFER, configSettings->amplitudeThreshold);

    if (dmaTransfersProcessed > dmaTransfersToSkip) {

        writeIndicator[writeBuffer] |= thresholdExceeded;

        writeBufferIndex += NUMBER_OF_SAMPLES_IN_DMA_TRANSFER / configSettings->sampleRateDivider;

        if (writeBufferIndex == NUMBER_OF_SAMPLES_IN_BUFFER) {

            writeBufferIndex = 0;

            writeBuffer = (writeBuffer + 1) & (NUMBER_OF_BUFFERS - 1);

            writeIndicator[writeBuffer] = false;

        }

    }

    dmaTransfersProcessed += 1;

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

    /* Copy the USB packet contents to the back-up register data structure location */

    copyToBackupDomain((uint32_t*)configSettings,  receiveBuffer + 1, sizeof(configSettings_t));

    /* Copy the back-up register data structure to the USB packet */

    copyFromBackupDomain(transmitBuffer + 1, (uint32_t*)configSettings, sizeof(configSettings_t));

    /* Set the time */

    AudioMoth_setTime(configSettings->time, 0);

}

/* Encode the compression buffer */

static void encodeCompressionBuffer(uint32_t numberOfCompressedBuffers) {

    for (uint32_t i = 0; i < UINT32_SIZE_IN_BITS; i += 1) {

        compressionBuffer[i] = numberOfCompressedBuffers & 0x01 ? 1 : -1;

        numberOfCompressedBuffers >>= 1;

    }

    for (uint32_t i = UINT32_SIZE_IN_BITS; i < COMPRESSION_BUFFER_SIZE_IN_BYTES / NUMBER_OF_BYTES_IN_SAMPLE; i += 1) {

        compressionBuffer[i] = 0;

    }

}

/* Save recording to SD card */

static AM_recordingState_t makeRecording(uint32_t currentTime, uint32_t recordDuration, bool enableLED, AM_extendedBatteryState_t extendedBatteryState, int32_t temperature) {

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

    if (configSettings->lowerFilterFreq == 0 && configSettings->higherFilterFreq == 0) {

        requestedFilterType = NO_FILTER;

        DigitalFilter_designHighPassFilter(effectiveSampleRate, DC_BLOCKING_FREQ);

    } else if (configSettings->lowerFilterFreq == UINT16_MAX) {

        requestedFilterType = LOW_PASS_FILTER;

        DigitalFilter_designBandPassFilter(effectiveSampleRate, DC_BLOCKING_FREQ, FILTER_FREQ_MULTIPLIER * configSettings->higherFilterFreq);

    } else if (configSettings->higherFilterFreq == UINT16_MAX) {

        requestedFilterType = HIGH_PASS_FILTER;

        DigitalFilter_designHighPassFilter(effectiveSampleRate, MAX(DC_BLOCKING_FREQ, FILTER_FREQ_MULTIPLIER * configSettings->lowerFilterFreq));

    } else {

        requestedFilterType = BAND_PASS_FILTER;

        DigitalFilter_designBandPassFilter(effectiveSampleRate, MAX(DC_BLOCKING_FREQ, FILTER_FREQ_MULTIPLIER * configSettings->lowerFilterFreq), FILTER_FREQ_MULTIPLIER * configSettings->higherFilterFreq);

    }

    /* Calculate the sample multiplier */

    float sampleMultiplier = 16.0f / (float)(configSettings->oversampleRate * configSettings->sampleRateDivider);

    DigitalFilter_applyAdditionalGain(sampleMultiplier);

    /* Set up the DMA transfers to skip */

    dmaTransfersProcessed = 0;

    dmaTransfersToSkip = configSettings->sampleRate / FRACTION_OF_SECOND_FOR_WARMUP / NUMBER_OF_SAMPLES_IN_DMA_TRANSFER;

    /* Calculate recording parameters */

    uint32_t numberOfSamplesInHeader = sizeof(wavHeader) / NUMBER_OF_BYTES_IN_SAMPLE;

    uint32_t maximumNumberOfSeconds = (MAXIMUM_WAV_FILE_SIZE - sizeof(wavHeader)) / NUMBER_OF_BYTES_IN_SAMPLE / effectiveSampleRate;

    bool fileSizeLimited = (recordDuration > maximumNumberOfSeconds);

    uint32_t numberOfSamples = effectiveSampleRate * (fileSizeLimited ? maximumNumberOfSeconds : recordDuration);

    /* Initialise microphone for recording */

    AudioMoth_enableExternalSRAM();

    AudioMoth_enableMicrophone(configSettings->gain, configSettings->clockDivider, configSettings->acquisitionCycles, configSettings->oversampleRate);

    AudioMoth_initialiseDirectMemoryAccess(primaryBuffer, secondaryBuffer, NUMBER_OF_SAMPLES_IN_DMA_TRANSFER);

    AudioMoth_startMicrophoneSamples(configSettings->sampleRate);

    /* Show LED for SD card activity */
   
    if (enableLED) AudioMoth_setRedLED(true);

    /* Open a file with the current local time as the name */

    time_t rawtime = currentTime + configSettings->timezoneHours * SECONDS_IN_HOUR + configSettings->timezoneMinutes * SECONDS_IN_MINUTE;

    struct tm *time = gmtime(&rawtime);

    uint32_t length = sprintf(fileName, "%04d%02d%02d_%02d%02d%02d", 1900 + time->tm_year, time->tm_mon + 1, time->tm_mday, time->tm_hour, time->tm_min, time->tm_sec);

    char *extension = configSettings->amplitudeThreshold > 0 ? "T.WAV" : ".WAV";

    strcpy(fileName + length, extension);

    FLASH_LED_AND_RETURN_ON_ERROR(AudioMoth_openFile(fileName));

    AudioMoth_setRedLED(false);

    /* Termination conditions */

    switchPositionChanged = false;

    bool supplyVoltageLow = false;

    /* Main record loop */

    uint32_t readBuffer = 0;

    uint32_t samplesWritten = 0;

    uint32_t buffersProcessed = 0;

    uint32_t numberOfCompressedBuffers = 0;

    uint32_t totalNumberOfCompressedSamples = 0;

    /* Ensure main loop doesn't start if the last buffer is currently being written to */

    while (writeBuffer == NUMBER_OF_BUFFERS - 1) { }

    /* Main recording loop */

    while (samplesWritten < numberOfSamples + numberOfSamplesInHeader && !switchPositionChanged && !supplyVoltageLow) {

        while (readBuffer != writeBuffer && samplesWritten < numberOfSamples + numberOfSamplesInHeader && !switchPositionChanged && !supplyVoltageLow) {

            /* Write the appropriate number of bytes to the SD card */

            uint32_t numberOfSamplesToWrite = MIN(numberOfSamples + numberOfSamplesInHeader - samplesWritten, NUMBER_OF_SAMPLES_IN_BUFFER);

            if (!writeIndicator[readBuffer] && buffersProcessed > 0 && numberOfSamplesToWrite == NUMBER_OF_SAMPLES_IN_BUFFER) {

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

                /* Write the buffer */

                FLASH_LED_AND_RETURN_ON_ERROR(AudioMoth_writeToFile(buffers[readBuffer], NUMBER_OF_BYTES_IN_SAMPLE * numberOfSamplesToWrite));

                /* Clear LED */

                AudioMoth_setRedLED(false);

            }

            /* Increment buffer counters */

            readBuffer = (readBuffer + 1) & (NUMBER_OF_BUFFERS - 1);

            samplesWritten += numberOfSamplesToWrite;

            buffersProcessed += 1;

        }

        /* Check the voltage level */

        if (configSettings->enableLowVoltageCutoff && !AudioMoth_isSupplyAboveThreshold()) {

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

    /* Initialise the WAV header */

    samplesWritten = MAX(numberOfSamplesInHeader, samplesWritten);

    setHeaderDetails(&wavHeader, effectiveSampleRate, samplesWritten - numberOfSamplesInHeader - totalNumberOfCompressedSamples);

    setHeaderComment(&wavHeader, currentTime, configSettings->timezoneHours, configSettings->timezoneMinutes, (uint8_t*)AM_UNIQUE_ID_START_ADDRESS, configSettings->gain, extendedBatteryState, temperature, switchPositionChanged, supplyVoltageLow, fileSizeLimited, configSettings->amplitudeThreshold, requestedFilterType, configSettings->lowerFilterFreq, configSettings->higherFilterFreq);

    /* Write the header */

    if (enableLED) AudioMoth_setRedLED(true);

    FLASH_LED_AND_RETURN_ON_ERROR(AudioMoth_seekInFile(0));

    FLASH_LED_AND_RETURN_ON_ERROR(AudioMoth_writeToFile(&wavHeader, sizeof(wavHeader)));

    /* Close the file */

    FLASH_LED_AND_RETURN_ON_ERROR(AudioMoth_closeFile());

    AudioMoth_setRedLED(false);

    /* Return with state */

    if (switchPositionChanged) return SWITCH_CHANGED;

    if (supplyVoltageLow) return SUPPLY_VOLTAGE_LOW;

    if (fileSizeLimited) return FILE_SIZE_LIMITED;

    return RECORDING_OKAY;

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

    time_t rawtime = currentTime;

    struct tm *time = gmtime(&rawtime);

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

static void flashLedToIndicateBatteryLife(void){

    uint32_t numberOfFlashes = LOW_BATTERY_LED_FLASHES;

    uint32_t supplyVoltage = AudioMoth_getSupplyVoltage();

    AM_batteryState_t batteryState = AudioMoth_getBatteryState(supplyVoltage);

    /* Set number of flashes according to battery state */

    if (batteryState > AM_BATTERY_LOW) {

        numberOfFlashes = (batteryState >= AM_BATTERY_4V6) ? 4 : (batteryState >= AM_BATTERY_4V4) ? 3 : (batteryState >= AM_BATTERY_4V0) ? 2 : 1;

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

