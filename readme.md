# picoACU
## Project Objectives
The aim is to create an antenna control unit (ACU) for a dish antenna that will track various Earth imaging satellites.
The heart of the ACU is a Raspberry PI pico that takes input (i.e. azimuth and elevation angles) from a remote PC running the satellite tracking and data processing software, and also from an IMU which provides feedback of the antenna's actual azimuth and elevation. The pico will output commands to stepper motor drivers using a PID algorythm with the aim of zeroing the difference between the commanded angles and the measured angles.
Communications between the pico and the remote PC, and the IMU will likely be over a RS485 link using some implementation of the Easycom protocol. 
The programming is in C
## Dependencies
The project is built upon the [pico-SDK](https://github.com/raspberrypi/pico-sdk) from the Raspberry PI foundation.
I created the project using the python script [pico-project-generator](https://github.com/raspberrypi/pico-project-generator) also from the Raspberry PI foundation's github repos.
The only changes to the project created by the script is to edit the C  code and to tweak the CMakeLists.txt file to include the target libraries.
