# ojousima.logger.c
Data logging firmware for RuuviTag / nRF52. Built on top of Nordic SDK 15.2 and ruuvi.firmware.c, uses both Ruuvi and external repositories as submodules.

# Setting up
## SDK 15.2
Download [Nordic SDK15.2](https://developer.nordicsemi.com/nRF5_SDK/nRF5_SDK_v15.x.x/) and install it one level above the project root. 

## Submodules
Run `git submodule update --init --recursive`. This will search for and install the other git repositories referenced by this project.

## Toolchain
Segger Embedded Studio can be set up by following this [RuuviLab Tool post](https://lab.ruuvi.com/ses/).

# Usage
Compile and flash the project to your board using SES or flash the released hex.
*Important: The .hex file has only softdevice + application, there is no bootloader.*

Note: You should erase your board entirely in case there is a bootloader from a previous firmware.

Connect to the tag with application able to register to Nordic UART Service TX notifications, for example
Nordic Toolbox UART app. Data is sent automatically in in batches of 6 8-bit X-Y-Z values.
Export the logs.

Values are signed integers, LSB has scale of 4 mG. For example 
`FF-FF-FF-00-00-FE-00-00-00-00-00-00-00-00-00-00-00-00` 

is interpreted as
```
FF-FF-FF
00-00-FE
00-00-00
00-00-00
00-00-00
00-00-00
```

```
-4,-4,-4
 0, 0,-8
 0, 0, 0
 0, 0, 0
 0, 0, 0
 0, 0, 0
```

For further reading, please see this [blog post](https://blog.ruuvi.com/monitoring-motor-operation-with-ruuvitag-df1a5739a926)

# Power consumption
This firmware is for testing and development only, and the firmware is not power optimized in any manner. 

# How to contribute
Open an issue on Github if you want to see some new feature or fix a bug. 

# Licensing
Ruuvi code is BSD-3 licensed. Submodules and external dependencies have their own licenses, which are BSD-compatible.

# Changelog
## 1.0.0 
 - Fork Ruuvi firmware
 - Stream acceleration data over GATT