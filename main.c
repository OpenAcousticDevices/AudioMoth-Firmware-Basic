/****************************************************************************
 * main.c
 * openacousticdevices.info
 * June 2017
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

/* WAVE header constant */

#define PCM_FORMAT                          1
#define RIFF_ID_LENGTH                      4
#define LENGTH_OF_COMMENT                   128

/* USB configuration constant */

#define MAX_START_STOP_PERIODS              5

/* DC filter constant */

#define DC_BLOCKING_BITS                    8

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

#define MAX(a,b) (((a) (b)) ? (a) : (b))

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

void setHeaderComment(uint32_t currentTime, uint8_t *serialNumber, uint32_t gain) {

    time_t rawtime = currentTime;

    struct tm *time = gmtime(&rawtime);

    char *comment = wavHeader.icmt.comment;

    AM_batteryState_t batteryState = AudioMoth_getBatteryState();

    sprintf(comment, "Recorded at %02d:%02d:%02d %02d/%02d/%04d by AudioMoth %08X%08X at gain setting %d while battery state was ",
            time->tm_hour, time->tm_min, time->tm_sec, time->tm_mday, 1 + time->tm_mon, 1900 + time->tm_year,
            (unsigned int)(serialNumber + 8), (unsigned int)serialNumber, (unsigned int)gain);

    comment += 104;

    if (batteryState == AM_BATTERY_LOW) {

        sprintf(comment, "< 3.6V");

    } else if (batteryState >= AM_BATTERY_FULL) {

        sprintf(comment, "> 5.0V");

    } else {

        batteryState += 35;

        int tens = batteryState / 10;
        int units = batteryState - 10 * tens;

        sprintf(comment, "%01d.%02dV", tens, units);

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
    uint8_t clockBand;
    uint8_t clockDivider;
    uint8_t acquisitionCycles;
    uint8_t oversampleRate;
    uint32_t sampleRate;
    uint16_t sleepDuration;
    uint16_t recordDuration;
    uint8_t enableLED;
    uint8_t activeStartStopPeriods;
    startStopPeriod_t startStopPeriods[MAX_START_STOP_PERIODS];
} configSettings_t;

#pragma pack(pop)

configSettings_t defaultConfigSettings = {
    .time = 0,
    .gain = 2,
    .clockBand = 4,
    .clockDivider = 2,
    .acquisitionCycles = 2,
    .oversampleRate = 16,
    .sampleRate = 48000,
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
    }
};

uint32_t *previousSwitchPosition = (uint32_t*)AM_BACKUP_DOMAIN_START_ADDRESS;

uint32_t *timeOfNextRecording = (uint32_t*)(AM_BACKUP_DOMAIN_START_ADDRESS + 4);

uint32_t *durationOfNextRecording = (uint32_t*)(AM_BACKUP_DOMAIN_START_ADDRESS + 8);

configSettings_t *configSettings = (configSettings_t*)(AM_BACKUP_DOMAIN_START_ADDRESS + 12);

/* DC filter variables */

static int32_t previousSample;
static int32_t previousFilterOutput;

/* SRAM buffer variables */

static volatile uint32_t writeBuffer;
static volatile uint32_t writeBufferIndex;

static volatile bool recordingCancelled;

static int16_t* buffers[NUMBER_OF_BUFFERS];

/* Current recording file name */

static char fileName[13];

/* Function prototypes */

static void filter(int16_t* data, int size);
static void flashLedToIndicateBatteryLife(void);
static void makeRecording(uint32_t currentTime, uint32_t recordDuration, bool enableLED);
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

/* AudioMoth handlers */

inline void AudioMoth_handleSwitchInterrupt() {

    recordingCancelled = true;

}

inline void AudioMoth_handleMicrophoneInterrupt(int16_t sample) { }

inline void AudioMoth_handleDirectMemoryAccessInterrupt(bool primaryChannel, int16_t **nextBuffer) {

    /* Update the current buffer index and write buffer */

    writeBufferIndex += NUMBER_OF_SAMPLES_IN_DMA_TRANSFER;

    if (writeBufferIndex == NUMBER_OF_SAMPLES_IN_BUFFER) {

        writeBufferIndex = 0;

        writeBuffer = (writeBuffer + 1) & (NUMBER_OF_BUFFERS - 1);

    }

    /* Update the next buffer index and write buffer */

    int nextWriteBuffer = writeBuffer;

    int nextWriteBufferIndex = writeBufferIndex + NUMBER_OF_SAMPLES_IN_DMA_TRANSFER;

    if (nextWriteBufferIndex == NUMBER_OF_SAMPLES_IN_BUFFER) {

        nextWriteBufferIndex = 0;

        nextWriteBuffer = (nextWriteBuffer + 1) & (NUMBER_OF_BUFFERS - 1);

    }

    /* Re-activate the DMA */

    *nextBuffer = buffers[nextWriteBuffer] + nextWriteBufferIndex;

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

static void filter(int16_t *data, int size) {

    /* Calculate the multiplier to normalise the volume across different over sampling rates */

    uint32_t multiplier = 16 / configSettings->oversampleRate;

    if (multiplier == 0) {
        multiplier = 1;
    }

    /* DC filter */

    int32_t filteredOutput;
    int32_t scaledPreviousFilterOutput;

    for (int i = 0; i < size; i++) {

        int16_t sample = multiplier * data[i];

        scaledPreviousFilterOutput = (int32_t)(0.995f * previousFilterOutput);

        filteredOutput = sample - previousSample + scaledPreviousFilterOutput;

        data[i] = (int16_t)filteredOutput;

        previousFilterOutput = filteredOutput;

        previousSample = (int32_t)sample;

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

    /* Switch to HFRCO */

    if (configSettings->clockBand < AM_HFXO) {

        AudioMoth_enableHFRCO(configSettings->clockBand);

        uint32_t clockFrequency = AudioMoth_getClockFrequency(configSettings->clockBand);

        uint32_t actualSampleRate = AudioMoth_calculateSampleRate(clockFrequency, configSettings->clockDivider, configSettings->acquisitionCycles, configSettings->oversampleRate);

        uint32_t targetFrequency = (float)clockFrequency * (float)configSettings->sampleRate / (float)actualSampleRate;

        AudioMoth_calibrateHFRCO(targetFrequency);

        AudioMoth_selectHFRCO();

    }

    /* Calculate recording parameters */

    uint32_t numberOfSamples = configSettings->sampleRate * recordDuration;

    /* Initialise the WAV header */

    setHeaderDetails(configSettings->sampleRate, numberOfSamples);

    setHeaderComment(currentTime, (uint8_t*)AM_UNIQUE_ID_START_ADDRESS, configSettings->gain);

    /* Initialise microphone for recording */

    AudioMoth_enableExternalSRAM();

    AudioMoth_enableMicrophone(configSettings->gain, configSettings->clockDivider, configSettings->acquisitionCycles, configSettings->oversampleRate);

    AudioMoth_initialiseDirectMemoryAccess(buffers[0], buffers[0] + NUMBER_OF_SAMPLES_IN_DMA_TRANSFER, NUMBER_OF_SAMPLES_IN_DMA_TRANSFER);

    AudioMoth_startMicrophoneSamples();

    /* Initialise file system and open a new file */

    RETURN_ON_ERROR(AudioMoth_enableFileSystem());

    /* Open a file with the name as a UNIX time stamp in HEX */

    sprintf(fileName, "%08X.WAV", (unsigned int)currentTime);

    RETURN_ON_ERROR(AudioMoth_openFile(fileName));

    /* Main record loop */

    uint32_t samplesWritten = 0;

    uint32_t buffersProcessed = 0;

    uint32_t readBuffer = writeBuffer;

    while (samplesWritten < numberOfSamples && !recordingCancelled) {

        while (readBuffer != writeBuffer && samplesWritten < numberOfSamples && !recordingCancelled) {

            /* Light LED during SD card write if appropriate */

            if (enableLED) {

                AudioMoth_setRedLED(true);

            }

            /* DC filter the microphone samples */

            filter(buffers[readBuffer], NUMBER_OF_SAMPLES_IN_BUFFER);

            /* Write header to the first buffer */

            uint32_t bytesOccupiedByHeader = 0;

            if (buffersProcessed == NUMBER_OF_BUFFERS_TO_SKIP) {

                bytesOccupiedByHeader = sizeof(wavHeader);

                memcpy(buffers[readBuffer], &wavHeader, bytesOccupiedByHeader);

            }

            /* Write the appropriate number of bytes to the SD card */

            uint32_t numberOfBytesToWrite = 0;

            if (buffersProcessed >= NUMBER_OF_BUFFERS_TO_SKIP) {

                numberOfBytesToWrite = MIN(bytesOccupiedByHeader + 2 * (numberOfSamples - samplesWritten), 2 * NUMBER_OF_SAMPLES_IN_BUFFER);

            }

            AudioMoth_writeToFile(buffers[readBuffer], numberOfBytesToWrite);

            /* Increment buffer counters */

            readBuffer = (readBuffer + 1) & (NUMBER_OF_BUFFERS - 1);

            samplesWritten += (numberOfBytesToWrite - bytesOccupiedByHeader) >> 1;

            buffersProcessed += 1;

            /* Clear LED */

            AudioMoth_setRedLED(false);

        }

        /* Sleep until next DMA transfer is complete */

        AudioMoth_sleep();

    }

    /* Fix header if recording cancelled */

    if (samplesWritten < numberOfSamples) {

        setHeaderDetails(configSettings->sampleRate, samplesWritten);

        AudioMoth_seekInFile(0);

        AudioMoth_writeToFile(&wavHeader, sizeof(wavHeader));

    }

    /* Close the file */

    RETURN_ON_ERROR(AudioMoth_closeFile());

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

