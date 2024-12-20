# AudioMoth-Firmware-Basic

Firmware for AudioMoth devices, used in conjunction with the AudioMoth-Project framework, to produce the standard AudioMoth releases.

Compatible with the [AudioMoth Configuration App](https://github.com/OpenAcousticDevices/AudioMoth-Configuration-App). For usage instructions, visit [Open Acoustic Devices](https://www.openacousticdevices.info/getting-started).

### Usage ####

Clone the contents of [AudioMoth-Project](https://github.com/OpenAcousticDevices/AudioMoth-Project).

Replace the ```src/main.c``` from AudioMoth-Project with the ```src/main.c``` from this repository. Put all the remaining ```src/*.c``` files and all the ```src/*.h``` files from this repository into the ```/src/``` and ```/inc/``` folders of the AudioMoth-Project repository. Add the  ```/gps/``` folder from the AudioMoth-Project code into the compilation chain by updating the definitions of the include and source files in the ```/build/Makefile``` as below:

```
INC = ../cmsis ../device/inc ../emlib/inc ../emusb/inc ../drivers/inc ../fatfs/inc  ../gps/inc ../inc
SRC = ../device/src ../emlib/src ../emusb/src ../drivers/src ../fatfs/src  ../gps/src ../src
```

### Documentation ####

See the [Wiki](https://github.com/OpenAcousticDevices/AudioMoth-Firmware-Basic/wiki/AudioMoth) for a detailed description of the example code.

### License ###

Copyright 2017 [Open Acoustic Devices](http://www.openacousticdevices.info/).

[MIT license](http://www.openacousticdevices.info/license).
