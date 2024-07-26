import time, math
from machine import Pin, SPI
from breakout_paa5100 import BreakoutPAA5100

class PAA5100JE:
    def __init__(self, spi_id, sck_pin, mosi_pin, miso_pin, cs_pin):
        self.spi = SPI(spi_id, baudrate=9600, polarity=0, phase=0, bits=16, firstbit=SPI.MSB,
                       sck=Pin(sck_pin), mosi=Pin(mosi_pin), miso=Pin(miso_pin))
        self.cs = Pin(cs_pin, Pin.OUT)
        
        self.flo = BreakoutPAA5100(spi_id, cs_pin, sck_pin, mosi_pin, miso_pin)
        self.flo.set_rotation(BreakoutPAA5100.DEGREES_0)
        self.x = 0
        self.y = 0
    
    def send_values(self, values):
        self.cs.value(0)  # Select the device
        for value in values:
            self.spi.write(value.to_bytes(2, 'big'))  # Sending each value as 2 bytes
        self.cs.value(1)  # Deselect the device

    def read_motion(self):
    # Read x and y coordinates from PAA5100JE sensor
        delta = self.flo.get_motion()
        if delta is not None:
            self.x, self.y = delta
        else:
            self.x, self.y = 0
        return self.x, self.y

    def shut_down(self, deinitSPI=True):
        self.cs.value(1) # Deselect the device
        if deinitSPI:
            self.spi.deinit()
        
class MotionSensor:
    def __init__(self, motSen_x, motSen_y):
        self._stop = False
        self._thread = None
        
        self.motSen_x = motSen_x
        self.motSen_y = motSen_y
        
        self.prev_x = 0
        self.prev_y = 0
        
    def record(self):
        # Units are in millimeters
        current_motion_x = self.motSen_x.read_motion()
        current_motion_y = self.motSen_y.read_motion()
        
        if current_motion_x is not None and current_motion_y is not None:
            self.curr_x = current_motion_x[0]
            self.curr_y = current_motion_y[1]
            
            disp = self.displacement()
            angle = self.tilt_angle()
            
            # Update previous coordinates
            self.prev_x = self.curr_x
            self.prev_y = self.curr_y
        
        print(f"x coordinate: {self.curr_x:>10.5f} | y coordinate: {self.curr_y:>10.5f} | Displacement: {disp:>12.5f} | Tilt angle: {angle:>11.5f}")
        
        # Send data to main microcontroller through SPI
        #spi_connect = PAA5100JE(###)
        #self.spi_connect.send_values([self.curr_x, self.curr_y, disp, angle])

        time.sleep(0.1)  # get data every 100ms
    
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
            
#   -------------------------------------------------------
if __name__ == "__main__":
    # Create SPI objects (ID, sck, mosi, miso, cs)
    motSen_x = PAA5100JE(0, 18, 19, 16, 17)
    motSen_y = PAA5100JE(1, 10, 11, 12, 13)
    
    motion_sensor = MotionSensor(motSen_x, motSen_y)
    motion_sensor.record()
    time.sleep(3)
    motSen_x.shut_down()
    motSen_y.shut_down()
