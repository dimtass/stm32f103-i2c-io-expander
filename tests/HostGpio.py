import sys
sys.path.insert(0, './')
import os
import io
import errno
from enum import IntEnum
from HelperFunctions import HelperFunctions 
from GpioDir import GpioDir
from time import sleep

API_VERSION = '0.1'
GPIO_PATH = '/sys/class/gpio'
GPIO_EXPORT = GPIO_PATH + '/export'

class HostGpio(object):
    def __init__(self):
        self.__helper = HelperFunctions()

    def api_version(self):
        return API_VERSION

    def __export(self, pin: int):
        path = '{}/gpio{}'.format(GPIO_PATH, pin)
        for i in range(10):
            try:
                # Must be O_NONBLOCK for use with epoll in edge
                # triggered mode
                os.open(path, os.O_RDONLY | os.O_NONBLOCK)
            except IOError as e:
                if e.errno == errno.ENOENT:
                    with io.open(GPIO_EXPORT, 'w+b') as f:
                        f.write(str(pin).encode('ascii'))
                elif e.errno == errno.EACCES:
                    sleep(i / 100)
                else:
                    raise

    def set_config(self, pin: int, dir: int, init_val: int):
        # Export pin
        self.__export(pin)
        # Set pin direction
        dir_str = 'in' if dir == GpioDir.GPIO_DIR_INPUT else 'out'
        path = '{}/gpio{}/direction'.format(GPIO_PATH, pin)
        with io.open(path, 'w+b') as f:
            f.write(dir_str.encode('ascii'))
        # Set pin value
        if dir == GpioDir.GPIO_DIR_OUTPUT:
            self.write_pin(pin, init_val)

    def write_pin(self, pin: int, value: int):
        bit_value = 0 if value == 0 else 1
        path = '{}/gpio{}/value'.format(GPIO_PATH, pin)
        with io.open(path, 'w+b') as f:
            f.write(str(bit_value).encode('ascii'))

    def read_pin(self, pin: int):
        path = '{}/gpio{}/value'.format(GPIO_PATH, pin)
        with io.open(path, 'r') as f:
            return f.read().strip()

def main():
    pin = 203
    gpio = HostGpio()
    gpio.set_config(pin, GpioDir.GPIO_DIR_INPUT, 0)
    value = gpio.read_pin(pin)
    print('Pin {} value: {}'.format(pin,value))

    # gpio.set_config(GPIO_NPI_NEO2.GPIOG11, GpioDir.GPIO_DIR_OUTPUT, 0)
    # gpio.set_pin(GPIO_NPI_NEO2.GPIOG11, 1)
    # sleep(5)
    # gpio.set_pin(GPIO_NPI_NEO2.GPIOG11, 0)

if __name__ == "__main__":
    main()