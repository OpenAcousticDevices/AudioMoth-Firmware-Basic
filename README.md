# AudioMoth-Firmware-Basic
Basic firmware for AudioMoth devices, usable in conjunction with the AudioMoth-Project framework.

Compatible with the [AudioMoth Configuration App](https://github.com/OpenAcousticDevices/AudioMoth-Configuration-App). For usage instructions, visit [Open Acoustic Devices](https://www.openacousticdevices.info/getting-started).

### Usage ####

Clone the contents of [AudioMoth-Project](https://github.com/OpenAcousticDevices/AudioMoth-Project).

Replace ```src/main.c``` with this ```src/main.c``` from this repository. Put all the remaining ```src/*.c``` files in the ```/src/``` folder and all the ```src/*.h``` files in the ```/inc/``` folder. Add the  ```/gps/``` folder from this repository to the source folder, and add it into the compile by updating the definitions of the include and source files in the ```/build/Makefile``` to include the new ```/gps/``` folder:

```
INC = ../cmsis ../device/inc ../emlib/inc ../emusb/inc ../drivers/inc ../fatfs/inc  ../gps/inc ../inc
SRC = ../device/src ../emlib/src ../emusb/src ../drivers/src ../fatfs/src  ../gps/src ../src
```

If building from the command line tools no further changes are necessary. However, if building using Simplicity Studio, make sure that the new ```/gps/``` folder appears in the project files, and then open the project 'Properties' under the 'File' menu, navigate to the 'Linker' and 'Ordering' and make sure that the standard math library '-lm' is at the bottom of the list.

### Documentation ####

See the [Wiki](https://github.com/OpenAcousticDevices/AudioMoth-Firmware-Basic/wiki/AudioMoth) for detailed description of the example code.

### License ###

Copyright 2017 [Open Acoustic Devices](http://www.openacousticdevices.info/).

[MIT license](http://www.openacousticdevices.info/license).
