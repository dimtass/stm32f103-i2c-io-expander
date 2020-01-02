STM32F103 I2C I/O Expander
----
This firmware is used on the stm32f103 and makes to function as an I2C
I/O expansion that can be used from any linux SBC with an I2C interface.
Currently are supported 8 GPIOs that are dynamic and can be set on any
available GPIO of the STM32, thus makes it very flexible. Therefore,
you each GPIO can be any pin of the STM32 and is programmable using
the I2C bus.

The default I2C address is `0x08` and this can  be changed in the code.

> This is just a demo and it's a work still on progress as there's no
kernel module yet.

This repo is part of this blog post here:
[https://www.stupid-projects.com/devops-for-embedded-part-3/](https://www.stupid-projects.com/devops-for-embedded-part-3/)


## Build
To build the firmware you need cmake and a GCC toolchain for ARM Cortex-M.
Usually I install all my toolchains in `/opt/toolchains/`, but in any case
you need to pass the path of the toolchain as an argument to the build script,
like this:

```sh
TOOLCHAIN_DIR=/opt/toolchains/gcc-arm-none-eabi-9-2019-q4-major CLEANBUILD=true USE_STDPERIPH_DRIVER=ON USE_DBGUART=ON SRC=src ./build.sh
```

The above command will build the firmware and the `build.sh` it's a generic
build script that I use in all my cmake STM32 firmwares. You can have a look
at it for the rest of the available options.

## Flash firmware
To flash the firmware the following command should be enough:

```sh
./flash.sh
```

In case that the command fails then try this:
```sh
st-flash --reset write build-stm32/stm32f103-i2c-io-expander.bin 0x8000000
```

## Usage
The I2C interface provides a set of registers that can be read and/or write
and program the GPIOs. The firmware currently supports only 8 GPIO I/Os.
This is a table of the available registers

Register | R/W | Function | Default value
-|-|-|-
0x00 | r/- | Magic byte 0 | 0xBE
0x01 | r/- | Magic byte 1 | 0xEF
0x02 | -/w | Activate configuration | -
0x03-0x0A | r/w | gpio[0:7] port register | 0xFF (GPIOA)
0x0B-0x12 | r/w | gpio[0:7] pin register | 0xFF (GPIO.0)
0x13-0x1A | r/w | gpio[0:7] configuration | 0xFF (see below)
0x1B-0x22 | r/w | gpio[0:7] value | 0xFF

> Note: generally `0xFF` means either `disabled` or `all` depending the case

#### Magic number
The first two registers hold the magic word which is: `0xBEEF`. You can use
this in order to probe the I2C bus fot this device and verify that exists.

#### Activation register
Register `0x02` is the activation register. This is used to activate the
configuration of a single or all gpios.

* If the value is the gpio index [0:MAX] then only that gpio will be initialized.
* If the value is `0xFF` then all the gpios will be initialized.

> Note: Be aware that initialization means that the
state of the gpio will be reset to the default one.

#### Port registers
Port registers hold the index of the port which is in the range of [0:5]
and the meaning of each number is the following:

Index | STM32 port
-|-
0 | GPIOA
1 | GPIOB
2 | GPIOC
3 | GPIOD
4 | GPIOE
5 | GPIOF

> Note: By default this value is set to `0xFF` which means disabled

#### Pin registers
The pin registers hold the index of the port pin that is assigned to the
gpio. The range is [0:15].

> Note: By default this value is set to `0xFF` which means disabled

#### Configuration registers
The configuration register is multi-functional and controls the following:

* `Pin direction`: Defines the pin direction

Value | Meaning
-|-
0 | Pin is set as input
1 | Pin is set as output

* `Initial value`: Defines the initial pin state when is set as output
and it can be `0` or `1`.

* `Output inverted`: This bit defines if the output is inverted and `0`
means that is not inverted and `1` means inverted.

The configuration value is 1 byte and only 3 bits have a meaning

b.7-b.3 | b.2 | b.1 | b.0
-|-|-|-
N/A | Inverted | Initial value | Direction

Therefore `0x03` means that the pin is output and by default is enabled.

## Run tests
This repo here contains some tests but in order to use them you need to
have set up a test server like the one is explained in this post
[here](https://www.stupid-projects.com/devops-for-embedded-part-3/). The
test server needs to have a GCC toolchain, python3 and robot-framework.

To run the simple robot test that programs the GPIOA.0 as output and then
sets it to HIGH, verifies the state, sets it to LOW and again verifies the
state, then you need to run this command:

```sh
robot --pythonpath . --outputdir ./results -v I2C_SLAVE:0x08 -v BOARD:nanopi_neo2 -t "Probe STM32" -t "Config output pin" -t "Set stm32 pin high" -t "Read host pin" -t "Reset stm32 pin" -t "Read host pin 2" ./robot-tests/
```

`BOARD` in this repo can be either `nanopi_neo2` or `nanopi_k1_plus` and
defines the pin index used for the pinmux gpios from the kernel. You can
define your own board file in a way similar to the examples and provide
the proper pin indexes that will be used with `sysfs`.

> Note: You need to run the above command inside the `tests/` folder.

There are several python scripts in the `tests/` folder:
* `STM32GpioExpander.py`: A class that implements the I2C command interface
for the STM32
* `HostGpio.py`: A class that implements the gpio interface for the SBC that
is used as a test server. The script uses `sysfs` to do that. Note that in
order to use this on any Linux SBC you need some additional udev rules (you
can have a look in the post mentioned in the README here).
* `GpioDir.py`: An enum definition for the gpio direction
* `GpioPin_nanopi_*.py`: These are pin definitions for each board. This is used
by the robot test and the specific SBC board is passed as an argument using the
`BOARD` variable.
* `HelperFunctions.py`: Some helper functions