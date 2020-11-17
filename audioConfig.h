/****************************************************************************
 * audioconfig.h
 * openacousticdevices.info
 * May 2020
 *****************************************************************************/

#ifndef __AUDIOCONFIG_H
#define __AUDIOCONFIG_H

typedef enum {AC_EVENT_PULSE, AC_EVENT_START, AC_EVENT_BYTE, AC_EVENT_BIT_ERROR, AC_EVENT_CRC_ERROR} AC_audioConfigurationEvent_t;

extern void AudioConfig_handleAudioConfigurationEvent(AC_audioConfigurationEvent_t event);

extern void AudioConfig_handleAudioConfigurationPacket(uint8_t *receiveBuffer, uint32_t size);

void AudioConfig_enableAudioConfiguration(void);

void AudioConfig_disableAudioConfiguration(void);

bool AudioConfig_listenForAudioConfigurationTone(uint32_t miliseconds);

bool AudioConfig_listenForAudioConfigurationPackets(bool timeout, uint32_t milliseconds);

void AudioConfig_cancelAudioConfiguration(void);

#endif /* __AUDIOCONFIG_H */
