import time
from machine import Pin, SPI
from breakout_paa5100 import BreakoutPAA5100

class SPIInterface:
    def __init__(self, sck_pin, mosi_pin, miso_pin, cs_pin):
        self.spi = SPI(0, baudrate=9600, polarity=0, phase=0, bits=8, firstbit=SPI.MSB,
                       sck=Pin(sck_pin), mosi=Pin(mosi_pin), miso=Pin(miso_pin))
        self.cs = Pin(cs_pin, Pin.OUT)

    def spi_write_read(self, write_data):
        self.cs.value(0)  # Select the device
        read_data = self.spi.readinto(write_data)
        self.cs.value(1)  # Deselect the device
        return read_data

class MotionSensor:
    def __init__(self):
        self.flo = BreakoutPAA5100()
        self.flo.set_rotation(BreakoutPAA5100.DEGREES_0)
        self.x = 0
        self.y = 0

    def read_motion(self):
        delta = self.flo.get_motion()
        if delta is not None:
            self.x, self.y = delta
            print("Relative: x {}, y {}".format(self.x, self.y))

if __name__ == "__main__":
    # SPI0 Pins (Default)
    SCK_PIN = 18
    MOSI_PIN = 19
    MISO_PIN = 16
    CS_PIN = 17

    spi_interface = SPIInterface(SCK_PIN, MOSI_PIN, MISO_PIN, CS_PIN)
    motion_sensor = MotionSensor()

    while True:
        motion_sensor.read_motion()
        time.sleep(0.1)
