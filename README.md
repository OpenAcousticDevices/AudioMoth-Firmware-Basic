# AudioMoth-Firmware-Basic
Basic firmware for AudioMoth devices, usable in conjunction with the AudioMoth-Project framework.

Compatible with the [AudioMoth Configuration App](https://github.com/OpenAcousticDevices/AudioMoth-Configuration-App). For usage instructions, visit [Open Acoustic Devices](https://www.openacousticdevices.info/getting-started).

### Usage ####

Clone the contents of [AudioMoth-Project](https://github.com/OpenAcousticDevices/AudioMoth-Project).

Replace ```src/main.c``` with this ```main.c``` from this repository. Put all the remaining ```.c``` files in the ```/src/``` folder and all the ```.h``` files in the ```/inc/``` folder. If building from the command line tools no further changes are necessary. However, if building using Simplicity Studio, open the project 'Properties' under the 'File' menu, navigate to the 'Linker' and 'Ordering' and make sure that the standard math library '-lm' is at the bottom of the list.

### Documentation ####

See the [Wiki](https://github.com/OpenAcousticDevices/AudioMoth-Firmware-Basic/wiki/AudioMoth) for detailed description of the example code.

### License ###

Copyright 2017 [Open Acoustic Devices](http://www.openacousticdevices.info/).

[MIT license](http://www.openacousticdevices.info/license).
