import time
from machine import Pin, SPI

# SPI0 Pins
SCK_PIN = 18
MOSI_PIN = 19
MISO_PIN = 16
CS_PIN = 17

# Initialize SPI
spi = SPI(0, baudrate=9600, polarity=0, phase=0, bits=8, firstbit=SPI.MSB,
          sck=Pin(SCK_PIN), mosi=Pin(MOSI_PIN), miso=Pin(MISO_PIN))

# Chip Select (CS) pin as output
cs = Pin(CS_PIN, Pin.OUT)

# Function to write and read data via SPI
def spi_write_read(write_data):
    cs.value(0)  # Select the device
    read_data = spi.readinto(write_data)
    cs.value(1)  # Deselect the device
    return read_data


# Pick *one* sensor type by uncommenting the relevant line below:

# PMW3901
#from breakout_pmw3901 import BreakoutPMW3901 as FlowSensor

# PAA5100
from breakout_paa5100 import BreakoutPAA5100 as FlowSensor

flo = FlowSensor()
flo.set_rotation(FlowSensor.DEGREES_0)

tx = 0
ty = 0
x = 0
y = 0

while True:
    delta = flo.get_motion()
    if delta is not None:
        x = delta[0]
        y = delta[1]
        tx += x
        ty += y
        #print("Relative: x {}, y {} | Absolute: tx {}, ty {}".format(x, y, tx, ty))
        print("Relative: x {}, y {}".format(x, y))
    time.sleep(0.1)