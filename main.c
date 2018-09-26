/****************************************************************************
 * main.c
 * openacousticdevices.info
 * Sept 2018
 *****************************************************************************/

#include <time.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#include "audioMoth.h"

/* Sleep and LED constants */

#define DEFAULT_WAIT_INTERVAL               1

#define WAITING_LED_FLASH_INTERVAL          2
#define WAITING_LED_FLASH_DURATION          1

#define LOW_BATTERY_LED_FLASHES             10

#define SHORT_LED_FLASH_DURATION            100
#define LONG_LED_FLASH_DURATION             500

/* Useful time constants */

#define SECONDS_IN_MINUTE                   60
#define SECONDS_IN_HOUR                     (60 * SECONDS_IN_MINUTE)
#define SECONDS_IN_DAY                      (24 * SECONDS_IN_HOUR)

/* SRAM buffer constants */

#define NUMBER_OF_BUFFERS                   8
#define EXTERNAL_SRAM_SIZE_IN_SAMPLES       (AM_EXTERNAL_SRAM_SIZE_IN_BYTES / 2)
#define NUMBER_OF_SAMPLES_IN_BUFFER         (EXTERNAL_SRAM_SIZE_IN_SAMPLES / NUMBER_OF_BUFFERS)
#define NUMBER_OF_SAMPLES_IN_DMA_TRANSFER   1024
#define NUMBER_OF_BUFFERS_TO_SKIP           1

/* WAV header constant */

#define PCM_FORMAT                          1
#define RIFF_ID_LENGTH                      4
#define LENGTH_OF_COMMENT                   128

/* USB configuration constant */

#define MAX_START_STOP_PERIODS              5

/* DC filter constant */

#define DC_BLOCKING_FACTOR                  0.995f

/* Useful macros */

#define FLASH_LED(led, duration) { \
    AudioMoth_set ## led ## LED(true); \
    AudioMoth_delay(duration); \
    AudioMoth_set ## led ## LED(false); \
}

#define RETURN_ON_ERROR(fn) { \
    bool success = (fn); \
    if (success != true) { \
        recordingCancelled = true; \
        FLASH_LED(Both, LONG_LED_FLASH_DURATION) \
        return; \
    } \
}

#define SAVE_SWITCH_POSITION_AND_POWER_DOWN(duration) { \
    *previousSwitchPosition = switchPosition; \
    AudioMoth_powerDownAndWake(duration, true); \
}

#define MAX(a,b) (((a) > (b)) ? (a) : (b))

#define MIN(a,b) (((a) < (b)) ? (a) : (b))

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
    chunk_t data;
} wavHeader_t;

#pragma pack(pop)

static wavHeader_t wavHeader = {
    .riff = {.id = "RIFF", .size = 0},
    .format = "WAVE",
    .fmt = {.id = "fmt ", .size = sizeof(wavFormat_t)},
    .wavFormat = {.format = PCM_FORMAT, .numberOfChannels = 1, .samplesPerSecond = 0, .bytesPerSecond = 0, .bytesPerCapture = 2, .bitsPerSample = 16},
    .list = {.id = "LIST", .size = RIFF_ID_LENGTH + sizeof(icmt_t)},
    .info = "INFO",
    .icmt = {.icmt.id = "ICMT", .icmt.size = LENGTH_OF_COMMENT, .comment = ""},
    .data = {.id = "data", .size = 0}
};

void setHeaderDetails(uint32_t sampleRate, uint32_t numberOfSamples) {

    wavHeader.wavFormat.samplesPerSecond = sampleRate;
    wavHeader.wavFormat.bytesPerSecond = 2 * sampleRate;
    wavHeader.data.size = 2 * numberOfSamples;
    wavHeader.riff.size = 2 * numberOfSamples + sizeof(wavHeader_t) - sizeof(chunk_t);

}

void setHeaderComment(uint32_t currentTime, int8_t timezone, uint8_t *serialNumber, uint32_t gain) {

    time_t rawtime = currentTime + timezone * SECONDS_IN_HOUR;

    struct tm *time = gmtime(&rawtime);

    char *comment = wavHeader.icmt.comment;

    AM_batteryState_t batteryState = AudioMoth_getBatteryState();

    sprintf(comment, "Recorded at %02d:%02d:%02d %02d/%02d/%04d (UTC", time->tm_hour, time->tm_min, time->tm_sec, time->tm_mday, 1 + time->tm_mon, 1900 + time->tm_year);

    comment += 36;

    if (timezone < 0) sprintf(comment, "%d", timezone);

    if (timezone > 0) sprintf(comment, "+%d", timezone);

    if (timezone < 0 || timezone > 0) comment += 2;

    if (timezone < -9 || timezone > 9) comment += 1;

    sprintf(comment, ") by AudioMoth %08X%08X at gain setting %d while battery state was ", (unsigned int)*((uint32_t*)serialNumber + 1), (unsigned int)*((uint32_t*)serialNumber), (unsigned int)gain);

    comment += 74;

    if (batteryState == AM_BATTERY_LOW) {

        sprintf(comment, "< 3.6V.");

    } else if (batteryState >= AM_BATTERY_FULL) {

        sprintf(comment, "> 5.0V.");

    } else {

        batteryState += 35;

        sprintf(comment, "%01d.%01dV.", batteryState / 10, batteryState % 10);

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
    int8_t timezone;
} configSettings_t;

#pragma pack(pop)

configSettings_t defaultConfigSettings = {
    .time = 0,
    .gain = 2,
    .clockDivider = 4,
    .acquisitionCycles = 16,
    .oversampleRate = 1,
    .sampleRate = 384000,
    .sampleRateDivider = 8,
    .sleepDuration = 0,
    .recordDuration = 60,
    .enableLED = 1,
    .activeStartStopPeriods = 0,
    .startStopPeriods = {
        {.startMinutes = 60, .stopMinutes = 120},
        {.startMinutes = 300, .stopMinutes = 420},
        {.startMinutes = 540, .stopMinutes = 600},
        {.startMinutes = 720, .stopMinutes = 780},
        {.startMinutes = 900, .stopMinutes = 960}
    },
    .timezone = 0
};

uint32_t *previousSwitchPosition = (uint32_t*)AM_BACKUP_DOMAIN_START_ADDRESS;

uint32_t *timeOfNextRecording = (uint32_t*)(AM_BACKUP_DOMAIN_START_ADDRESS + 4);

uint32_t *durationOfNextRecording = (uint32_t*)(AM_BACKUP_DOMAIN_START_ADDRESS + 8);

configSettings_t *configSettings = (configSettings_t*)(AM_BACKUP_DOMAIN_START_ADDRESS + 12);

/* DC filter variables */

static int8_t bitsToShift;

static int32_t previousSample;
static int32_t previousFilterOutput;

/* SRAM buffer variables */

static volatile uint32_t writeBuffer;
static volatile uint32_t writeBufferIndex;

static volatile bool recordingCancelled;

static int16_t* buffers[NUMBER_OF_BUFFERS];

/* DMA buffers */

static int16_t primaryBuffer[NUMBER_OF_SAMPLES_IN_DMA_TRANSFER];
static int16_t secondaryBuffer[NUMBER_OF_SAMPLES_IN_DMA_TRANSFER];

/* Current recording file name */

static char fileName[20];

/* Firmware version and description */

static uint8_t firmwareVersion[AM_FIRMWARE_VERSION_LENGTH] = {1, 2, 0};

static uint8_t firmwareDescription[AM_FIRMWARE_DESCRIPTION_LENGTH] = "AudioMoth-Firmware-Basic";

/* Function prototypes */

static void flashLedToIndicateBatteryLife(void);
static void makeRecording(uint32_t currentTime, uint32_t recordDuration, bool enableLED);
static void filter(int16_t *source, int16_t *dest, uint8_t sampleRateDivider, uint32_t size);
static void scheduleRecording(uint32_t currentTime, uint32_t *timeOfNextRecording, uint32_t *durationOfNextRecording);

/* Main function */

int main(void) {

    /* Initialise device */

    AudioMoth_initialise();

    AM_switchPosition_t switchPosition = AudioMoth_getSwitchPosition();

    if (AudioMoth_isInitialPowerUp()) {

        *timeOfNextRecording = 0;

        *durationOfNextRecording = 0;

        *previousSwitchPosition = AM_SWITCH_NONE;

        memcpy(configSettings, &defaultConfigSettings, sizeof(configSettings_t));

    } else {

        /* Indicate battery state is not initial power up and switch has been moved into USB */

        if (switchPosition != *previousSwitchPosition && switchPosition == AM_SWITCH_USB) {

            flashLedToIndicateBatteryLife();

        }

    }

    /* Handle the case that the switch is in USB position  */

    if (switchPosition == AM_SWITCH_USB) {

        AudioMoth_handleUSB();

        SAVE_SWITCH_POSITION_AND_POWER_DOWN(DEFAULT_WAIT_INTERVAL);

    }

    /* Handle the case that the switch is in CUSTOM position but the time has not been set */

    if (switchPosition == AM_SWITCH_CUSTOM && (AudioMoth_hasTimeBeenSet() == false || configSettings->activeStartStopPeriods == 0)) {

        FLASH_LED(Both, SHORT_LED_FLASH_DURATION)

        SAVE_SWITCH_POSITION_AND_POWER_DOWN(DEFAULT_WAIT_INTERVAL);

    }

    /* Calculate time of next recording if switch has changed position */

    uint32_t currentTime = AudioMoth_getTime();

    if (switchPosition != *previousSwitchPosition) {

         if (switchPosition == AM_SWITCH_DEFAULT) {

             /* Set parameters to start recording now */

             *timeOfNextRecording = currentTime;

             *durationOfNextRecording = configSettings->recordDuration;

         } else {

             /* Determine starting time and duration of next recording */

             scheduleRecording(currentTime, timeOfNextRecording, durationOfNextRecording);

         }

    }

    /* Make recording if appropriate */

    bool enableLED = (switchPosition == AM_SWITCH_DEFAULT) || configSettings->enableLED;

    if (currentTime >= *timeOfNextRecording) {

        makeRecording(currentTime, *durationOfNextRecording, enableLED);

		if (switchPosition == AM_SWITCH_DEFAULT) {

			/* Set parameters to start recording after sleep period */

			if (!recordingCancelled) {

				*timeOfNextRecording = currentTime + configSettings->recordDuration + configSettings->sleepDuration;

			}

		} else {

			/* Determine starting time and duration of next recording */

			scheduleRecording(currentTime, timeOfNextRecording, durationOfNextRecording);

		}

    } else if (enableLED) {

        /* Flash LED to indicate waiting */

        FLASH_LED(Green, WAITING_LED_FLASH_DURATION)

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

void AudioMoth_timezoneRequested(int8_t *timezone) {

    *timezone = configSettings->timezone;

}


/* AudioMoth interrupt handlers */

inline void AudioMoth_handleSwitchInterrupt() {

    recordingCancelled = true;

}

inline void AudioMoth_handleMicrophoneInterrupt(int16_t sample) { }

inline void AudioMoth_handleDirectMemoryAccessInterrupt(bool isPrimaryBuffer, int16_t **nextBuffer) {

    int16_t *source = secondaryBuffer;

    if (isPrimaryBuffer) source = primaryBuffer;

    /* Update the current buffer index and write buffer */

    filter(source, buffers[writeBuffer] + writeBufferIndex, configSettings->sampleRateDivider, NUMBER_OF_SAMPLES_IN_DMA_TRANSFER);

    writeBufferIndex += NUMBER_OF_SAMPLES_IN_DMA_TRANSFER / configSettings->sampleRateDivider;

    if (writeBufferIndex == NUMBER_OF_SAMPLES_IN_BUFFER) {

        writeBufferIndex = 0;

        writeBuffer = (writeBuffer + 1) & (NUMBER_OF_BUFFERS - 1);

    }

}

/* AudioMoth USB message handlers */

void AudioMoth_usbFirmwareVersionRequested(uint8_t **firmwareVersionPtr) {

    *firmwareVersionPtr = firmwareVersion;

}

void AudioMoth_usbFirmwareDescriptionRequested(uint8_t **firmwareDescriptionPtr) {

    *firmwareDescriptionPtr = firmwareDescription;

}

inline void AudioMoth_usbApplicationPacketRequested(uint32_t messageType, uint8_t *transmitBuffer, uint32_t size) {

    /* Copy the current time to the USB packet */

    uint32_t currentTime = AudioMoth_getTime();

    memcpy(transmitBuffer + 1, &currentTime, 4);

    /* Copy the unique ID to the USB packet */

    memcpy(transmitBuffer + 5, (uint8_t*)AM_UNIQUE_ID_START_ADDRESS, AM_UNIQUE_ID_SIZE_IN_BYTES);

    /* Copy the battery state to the USB packet */

    AM_batteryState_t batteryState = AudioMoth_getBatteryState();

    memcpy(transmitBuffer + 5 + AM_UNIQUE_ID_SIZE_IN_BYTES, &batteryState, 1);

    /* Copy the firmware version to the USB packet */

    memcpy(transmitBuffer + 6 + AM_UNIQUE_ID_SIZE_IN_BYTES, firmwareVersion, AM_FIRMWARE_VERSION_LENGTH);

    /* Copy the firmware description to the USB packet */

    memcpy(transmitBuffer + 6 + AM_UNIQUE_ID_SIZE_IN_BYTES + AM_FIRMWARE_VERSION_LENGTH, firmwareDescription, AM_FIRMWARE_DESCRIPTION_LENGTH);

}

inline void AudioMoth_usbApplicationPacketReceived(uint32_t messageType, uint8_t* receiveBuffer, uint8_t *transmitBuffer, uint32_t size) {

    /* Copy the USB packet contents to the back-up register data structure location */

    memcpy(configSettings, receiveBuffer + 1, sizeof(configSettings_t));

    /* Copy the back-up register data structure to the USB packet */

    memcpy(transmitBuffer + 1, configSettings, sizeof(configSettings_t));

    /* Set the time */

    AudioMoth_setTime(configSettings->time);

}

/* Remove DC offset from the microphone samples */

static void filter(int16_t *source, int16_t *dest, uint8_t sampleRateDivider, uint32_t size) {

    int32_t filteredOutput;
    int32_t scaledPreviousFilterOutput;

    int index = 0;

    for (int i = 0; i < size; i += sampleRateDivider) {

        int32_t sample = 0;

        for (int j = 0; j < sampleRateDivider; j += 1) {

            sample += source[i + j];

        }

        if (bitsToShift > 0) {

            sample = sample >> bitsToShift;

        }

        scaledPreviousFilterOutput = (int32_t)(DC_BLOCKING_FACTOR * previousFilterOutput);

        filteredOutput = sample - previousSample + scaledPreviousFilterOutput;

        if (filteredOutput > INT16_MAX) {

            dest[index++] = INT16_MAX;

        } else if (filteredOutput < INT16_MIN) {

            dest[index++] = INT16_MIN;

        } else {

            dest[index++] = (int16_t)filteredOutput;

        }

        previousFilterOutput = filteredOutput;

        previousSample = sample;

    }

}

/* Save recording to SD card */

static void makeRecording(uint32_t currentTime, uint32_t recordDuration, bool enableLED) {

    /* Initialise buffers */

    writeBuffer = 0;

    writeBufferIndex = 0;

    recordingCancelled = false;

    buffers[0] = (int16_t*)AM_EXTERNAL_SRAM_START_ADDRESS;

    for (int i = 1; i < NUMBER_OF_BUFFERS; i += 1) {
        buffers[i] = buffers[i - 1] + NUMBER_OF_SAMPLES_IN_BUFFER;
    }

    /* Calculate the bits to shift */

    bitsToShift = 0;

    uint16_t oversampling = configSettings->oversampleRate * configSettings->sampleRateDivider;

    while (oversampling > 16) {
        oversampling = oversampling >> 1;
        bitsToShift += 1;
    }

    /* Calculate recording parameters */

    uint32_t numberOfSamplesInHeader = sizeof(wavHeader) >> 1;

    uint32_t numberOfSamples = configSettings->sampleRate / configSettings->sampleRateDivider * recordDuration;

    /* Initialise microphone for recording */

    AudioMoth_enableExternalSRAM();

    AudioMoth_enableMicrophone(configSettings->gain, configSettings->clockDivider, configSettings->acquisitionCycles, configSettings->oversampleRate);

    AudioMoth_initialiseDirectMemoryAccess(primaryBuffer, secondaryBuffer, NUMBER_OF_SAMPLES_IN_DMA_TRANSFER);

    AudioMoth_startMicrophoneSamples(configSettings->sampleRate);

    /* Initialise file system and open a new file */
   
    if (enableLED) {

        AudioMoth_setRedLED(true);

    }

    RETURN_ON_ERROR(AudioMoth_enableFileSystem());

    /* Open a file with the current local time as the name */

    time_t rawtime = currentTime + configSettings->timezone * SECONDS_IN_HOUR;

    struct tm *time = gmtime(&rawtime);

    sprintf(fileName, "%04d%02d%02d_%02d%02d%02d.WAV", 1900 + time->tm_year, time->tm_mon + 1, time->tm_mday, time->tm_hour, time->tm_min, time->tm_sec);

    RETURN_ON_ERROR(AudioMoth_openFile(fileName));

    AudioMoth_setRedLED(false);

    /* Main record loop */

    uint32_t samplesWritten = 0;

    uint32_t buffersProcessed = 0;

    uint32_t readBuffer = writeBuffer;

    while (samplesWritten < numberOfSamples + numberOfSamplesInHeader && !recordingCancelled) {

        while (readBuffer != writeBuffer && samplesWritten < numberOfSamples + numberOfSamplesInHeader && !recordingCancelled) {

            /* Light LED during SD card write if appropriate */

            if (enableLED) {

                AudioMoth_setRedLED(true);

            }

            /* Write the appropriate number of bytes to the SD card */

            uint32_t numberOfSamplesToWrite = 0;

            if (buffersProcessed >= NUMBER_OF_BUFFERS_TO_SKIP) {

                numberOfSamplesToWrite = MIN(numberOfSamples + numberOfSamplesInHeader - samplesWritten, NUMBER_OF_SAMPLES_IN_BUFFER);

            }

            RETURN_ON_ERROR(AudioMoth_writeToFile(buffers[readBuffer], 2 * numberOfSamplesToWrite));

            /* Increment buffer counters */

            readBuffer = (readBuffer + 1) & (NUMBER_OF_BUFFERS - 1);

            samplesWritten += numberOfSamplesToWrite;

            buffersProcessed += 1;

            /* Clear LED */

            AudioMoth_setRedLED(false);

        }

        /* Sleep until next DMA transfer is complete */

        AudioMoth_sleep();

    }

    /* Initialise the WAV header */

    samplesWritten = MAX(numberOfSamplesInHeader, samplesWritten);

    setHeaderDetails(configSettings->sampleRate / configSettings->sampleRateDivider, samplesWritten - numberOfSamplesInHeader);

    setHeaderComment(currentTime, configSettings->timezone, (uint8_t*)AM_UNIQUE_ID_START_ADDRESS, configSettings->gain);

    /* Write the header */

    if (enableLED) {

        AudioMoth_setRedLED(true);

    }

    RETURN_ON_ERROR(AudioMoth_seekInFile(0));

    RETURN_ON_ERROR(AudioMoth_writeToFile(&wavHeader, sizeof(wavHeader)));

    /* Close the file */

    RETURN_ON_ERROR(AudioMoth_closeFile());

    AudioMoth_setRedLED(false);

}

static void scheduleRecording(uint32_t currentTime, uint32_t *timeOfNextRecording, uint32_t *durationOfNextRecording) {

    /* Check number of active state stop periods */

    if (configSettings->activeStartStopPeriods > MAX_START_STOP_PERIODS) {

        configSettings->activeStartStopPeriods = MAX_START_STOP_PERIODS;

    }

    /* No active periods */

    if (configSettings->activeStartStopPeriods == 0) {

        *timeOfNextRecording = UINT32_MAX;

        *durationOfNextRecording = configSettings->recordDuration;

        return;

    }

    /* Calculate the number of seconds of this day */

    time_t rawtime = currentTime;

    struct tm *time = gmtime(&rawtime);

    uint32_t currentSeconds = SECONDS_IN_HOUR * time->tm_hour + SECONDS_IN_MINUTE * time->tm_min + time->tm_sec;

    /* Check each active start stop period */

    uint32_t durationOfCycle = configSettings->recordDuration + configSettings->sleepDuration;

    for (uint32_t i = 0; i < configSettings->activeStartStopPeriods; i += 1) {

        startStopPeriod_t *period = configSettings->startStopPeriods + i;

        /* Calculate the start and stop time of the current period */

        uint32_t startSeconds = SECONDS_IN_MINUTE * period->startMinutes;

        uint32_t stopSeconds = SECONDS_IN_MINUTE * period->stopMinutes;

        /* Calculate time to next period or time to next start in this period */

        if (currentSeconds < startSeconds) {

            *timeOfNextRecording = currentTime + (startSeconds - currentSeconds);

            *durationOfNextRecording = MIN(configSettings->recordDuration, stopSeconds - startSeconds);

            return;

        } else if (currentSeconds < stopSeconds) {

            uint32_t cycles = (currentSeconds - startSeconds + durationOfCycle) / durationOfCycle;

            uint32_t secondsFromStartOfPeriod = cycles * durationOfCycle;

            if (secondsFromStartOfPeriod < stopSeconds - startSeconds) {

                *timeOfNextRecording = currentTime + (startSeconds - currentSeconds) + secondsFromStartOfPeriod;

                *durationOfNextRecording = MIN(configSettings->recordDuration, stopSeconds - startSeconds - secondsFromStartOfPeriod);

                return;

            }

        }

    }

    /* Calculate time until first period tomorrow */

    startStopPeriod_t *firstPeriod = configSettings->startStopPeriods;

    uint32_t startSeconds = SECONDS_IN_MINUTE * firstPeriod->startMinutes;

    uint32_t stopSeconds = SECONDS_IN_MINUTE * firstPeriod->stopMinutes;

    *timeOfNextRecording = currentTime + (SECONDS_IN_DAY - currentSeconds) + startSeconds;

    *durationOfNextRecording = MIN(configSettings->recordDuration, stopSeconds - startSeconds);

}

static void flashLedToIndicateBatteryLife(void){

    uint32_t numberOfFlashes = LOW_BATTERY_LED_FLASHES;

    AM_batteryState_t batteryState = AudioMoth_getBatteryState();

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

