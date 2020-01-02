import sys
import smbus
from time import sleep
from enum import IntEnum

API_VERSION = "1.0"
DEV_PREAMBLE = 0xBEEF
DEF_I2C_ADDR = 0x08

class RegIndex(IntEnum):
    I2C_REG_INDEX_PREAMBLE = 0
    I2C_REG_INDEX_ACTIVATE = I2C_REG_INDEX_PREAMBLE + 2
    I2C_REG_INDEX_IO_PORT = I2C_REG_INDEX_ACTIVATE + 1
    I2C_REG_INDEX_IO_PIN = I2C_REG_INDEX_IO_PORT + (8 * 1)
    I2C_REG_INDEX_IO_CONFIG = I2C_REG_INDEX_IO_PIN + (8 * 1)
    I2C_REG_INDEX_IO_VALUE = I2C_REG_INDEX_IO_CONFIG + (8 * 1)

class GpioDir(IntEnum):
    GPIO_DIR_INPUT = 0
    GPIO_DIR_OUTPUT = 1

class STM32GpioExpander(object):
    """
    This is object implements the I2C communication
    between the host and the STM32 device.

    @author: Dimitris Tassopoulos <dimtass@gmail.com>
    """

    def __init__(self, i2c_addr: int):
        """
        Initialialize the I2C IO expander
        :param i2c_addr: The I2C address of the IO expander
        """
        self.__i2c_addr = i2c_addr
        self.__bus = smbus.SMBus(0)

    def api_version(self):
        """
        Get the object API version
        :return: The API version string
        """
        return API_VERSION

    def probe(self):
        """
        Probe for existing IO expander devices on the bus
        :return: True if a device exists, else False
        """
        preamble = self.__bus.read_byte_data(self.__i2c_addr, 0) << 8
        preamble += self.__bus.read_byte_data(self.__i2c_addr, 1)
        return True if preamble == DEV_PREAMBLE else False

    def write(self, reg: int, byte: int):
        """
        Write any register with any byte value
        :param reg: The register offset
        :param byte: The byte to write to the register
        :return: None
        """
        return self.__bus.write_byte_data(self.__i2c_addr, reg, byte)

    def read(self, reg: int):
        """
        Read any register from the device
        :param reg: The register offset
        :return: None
        """
        return self.__bus.read_byte_data(self.__i2c_addr, reg)

    def set_config(self, index: int, port: int, pin: int, dir: int, init_val: int, invert: int):
        """
        Set the device configuration
        :param index: The I/O index [0:7]
        :param port: The port index [0:5] (0: GPOA, 1: GPOB, 2: GPOC, e.t.c.)
        :param pin: The pin index [0:15]
        :param dir: The pin direction [0:1] (0: Input, 1: output. See also IntEnum.GpioDir)
        :param init_val: The output pin initial value
        :param invert: Output pin inverted [0:1] (0: Not inverted, 1: Inverted)
        """
        # Set port
        reg = RegIndex.I2C_REG_INDEX_IO_PORT + index
        self.write(reg, port)
        print("1. {}={}".format(hex(reg), port))
        # Set pin
        reg = RegIndex.I2C_REG_INDEX_IO_PIN + index
        self.write(reg, pin)
        print("2. {}={}".format(hex(reg), pin))
        # Set dir, init_val and invert
        reg = RegIndex.I2C_REG_INDEX_IO_CONFIG + index
        val = (invert << 4) + (init_val << 2) + dir
        self.write(reg, val)
        print("3. {}={}".format(hex(reg), val))
        # Activate
        reg = RegIndex.I2C_REG_INDEX_ACTIVATE
        self.write(reg, index)
        print("4. {}={}".format(hex(reg), index))

    def set_pin(self, index: int, value: int):
        """
        Set the output pin value (only for outputs)
        :param index: The pin index [0:7]
        :param value: The output value [0:1]
        """
        reg = RegIndex.I2C_REG_INDEX_IO_VALUE + index
        print("5. {}={}".format(hex(reg), index))
        return self.write(reg, value)

def main():
    gpio = STM32GpioExpander(DEF_I2C_ADDR)
    print(gpio.api_version())
    exists = gpio.probe()
    print('probe: %s' % exists)
    gpio.set_config(0, 0, 0, GpioDir.GPIO_DIR_OUTPUT, 0, 0)
    # gpio.set_pin(0, 1)


if __name__ == "__main__":
    main()