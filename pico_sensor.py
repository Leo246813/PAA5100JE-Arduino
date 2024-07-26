import time, math
from machine import Pin, SPI
from breakout_paa5100 import BreakoutPAA5100

class SPIInterface:
    def __init__(self, spi_id, sck_pin, mosi_pin, miso_pin, cs_pin):
        self.spi = SPI(spi_id, baudrate=9600, polarity=0, phase=0, bits=16, firstbit=SPI.MSB,
                       sck=Pin(sck_pin), mosi=Pin(mosi_pin), miso=Pin(miso_pin))
        self.cs = Pin(cs_pin, Pin.OUT)
    
    def send_values(self, values):
        self.cs.value(0)  # Select the device
        for value in values:
            self.spi.write(value.to_bytes(2, 'big'))  # Sending each value as 2 bytes
        self.cs.value(1)  # Deselect the device
    
class MotionSensor:
    def __init__(self, spi, cs, sck, mosi, miso):
        self.flo = BreakoutPAA5100(spi, cs, sck, mosi, miso)
        self.flo.set_rotation(BreakoutPAA5100.DEGREES_0)
        self.x = 0
        self.y = 0

    def read_motion(self):
    # Read x and y coordinates from PAA5100JE sensor
        delta = self.flo.get_motion()
        if delta is not None:
            self.x, self.y = delta
        else:
            self.x, self.y = 0
        return self.x, self.y

class Movements:
    def __init__(self, prev_x, prev_y, curr_x, curr_y):
        self.prev_x = prev_x
        self.prev_y = prev_y
        self.curr_x = curr_x
        self.curr_y = curr_y
        
    def displacement(self):
    # Calculate displacement using the change of coordinates with time
        self.dx = self.curr_x - self.prev_x
        self.dy = self.curr_y - self.prev_y
        disp = math.sqrt(self.dx**2 + self.dy**2)
        return disp

    def tilt_angle(self):
    # Calculate tilt angle with respect to positive y axis (in degrees)
        if self.curr_y == 0:
            return 0
        angle = math.atan2(self.curr_x, self.curr_y) * 180 / math.pi
        return angle

if __name__ == "__main__":
    # SPI0 Pins (capture motion in x direction)
    SCK_PIN_x = 18
    MOSI_PIN_x = 19
    MISO_PIN_x = 16
    CS_PIN_x = 17
    
    # SPI1 Pins (capture motion in y direction)
    SCK_PIN_y = 10
    MOSI_PIN_y = 11
    MISO_PIN_y = 12
    CS_PIN_y = 13
    
    # Create SPI objects
    spi_interface_x = SPIInterface(0, SCK_PIN_x, MOSI_PIN_x, MISO_PIN_x, CS_PIN_x)
    spi_interface_y = SPIInterface(1, SCK_PIN_y, MOSI_PIN_y, MISO_PIN_y, CS_PIN_y)
    
    # Initiate motion sensor
    motion_sensor_x = MotionSensor(0, CS_PIN_x, SCK_PIN_x, MOSI_PIN_x, MISO_PIN_x)
    motion_sensor_y = MotionSensor(1, CS_PIN_y, SCK_PIN_y, MOSI_PIN_y, MISO_PIN_y)
    
    # Buffer time for system set up
    time.sleep(1)
    
    # Test
    # Define coordinates
    x_coords = []
    y_coords = []
    disps = []
    angles = []
    
    # Initialize previous coordinates
    prev_x = 0
    prev_y = 0
    
    for i in range(50):
        # Units are in millimeters
        current_motion_x = motion_sensor_x.read_motion()
        current_motion_y = motion_sensor_y.read_motion()
        
        if current_motion_x is not None and current_motion_y is not None:
            curr_x = current_motion_x[0]
            curr_y = current_motion_y[1]
            
            x_coords.append(curr_x)
            y_coords.append(curr_y)
            
            move_calc = Movements(prev_x, prev_y, curr_x, curr_y)
            disps.append(move_calc.displacement())
            angles.append(move_calc.tilt_angle())
            
            # Update previous coordinates
            prev_x = curr_x
            prev_y = curr_y
        
        time.sleep(0.1)
        
    for j in range(50):
        print(f"x coordinate: {x_coords[j]:>10.5f} | y coordinate: {y_coords[j]:>10.5f} | Displacement: {disps[j]:>12.5f} | Tilt angle: {angles[j]:>11.5f}")
    
    '''
    # Actual
    # Initialize previous coordinates
    prev_x = 0
    prev_y = 0
    
    while True:
        # Units are in millimeters
        current_motion_x = motion_sensor_x.read_motion()
        current_motion_y = motion_sensor_y.read_motion()
        
        if current_motion_x is not None and current_motion_y is not None:
            curr_x = current_motion_x[0]
            curr_y = current_motion_y[1]
            
            move_calc = Movements(prev_x, prev_y, curr_x, curr_y)
            disp = move_calc.displacement()
            angle = move_calc.tilt_angle()
            
            # Update previous coordinates
            prev_x = curr_x
            prev_y = curr_y
        
        print(f"x coordinate: {curr_x:>10.5f} | y coordinate: {curr_y:>10.5f} | Displacement: {disp:>12.5f} | Tilt angle: {angle:>11.5f}")
        
        # Connect to main microcontroller through SPI
        spi_connect = SPIInterface(0, , , , )
        spi_connect.send_values([curr_x, curr_y, disp, angle])

        time.sleep(0.01)  # get data every 10ms
    '''
