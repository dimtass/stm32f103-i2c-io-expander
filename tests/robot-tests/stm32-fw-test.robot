*** Variables ***
${I2C_SLAVE}     0x08
${INDEX} =       0
${PIN} =         0
${PORT} =        0
${IO} =          0
${BOARD}

*** Settings ***
Documentation          This test verifies that there is an STM32 board
...                    that runs the DUT firmware. This is done by reading
...                    the preamble
...                    ** This test suite must run in the TARGET board **
Library   STM32GpioExpander.STM32GpioExpander     ${8}   WITH NAME      stm32
Library   HostGpio.HostGpio   WITH NAME      host

*** Test Cases ***
Probe STM32
     ${probe} =     stm32.probe
     Log  STM32GpioExpander exists ${probe}

Config output pin
     ${config} =    stm32.set_config    ${0}  ${0}  ${0}  ${1}  ${0}  ${0}
     Log  Configured gpio[0] as output as GPOA PIN0

Set stm32 pin high
     ${pin} =  stm32.set_pin  ${0}  ${1}
     Log  Set gpio[0] to HIGH

Read host pin
     host.set_config     ${203}  ${0}  ${0}
     ${pin} =  host.read_pin  ${203}
     Should Be Equal     ${pin}  1
     Log  Read gpio[0] value: ${pin}

Reset stm32 pin
     ${pin} =  stm32.set_pin  ${0}  ${0}
     Log  Set gpio[0] to LOW

Read host pin 2
     host.set_config     ${203}  ${0}  ${0}
     ${pin} =  host.read_pin  ${203}
     Should Be Equal     ${pin}  0
     Log  Read gpio[0] value: ${pin}
