import time, math, gc
from machine import Pin, SPI
from pyControl.hardware import *
from breakout_paa5100 import BreakoutPAA5100
'''
ImportError: could not import pyControl module because RP pico doesn't have enough space
Changes: Digital_output changed to Pin (self.select.on/off --> value(0/1))
         Cannot test for Analog_input and related functions
         PAA5100JE does not have SROM ID, need to change way to interrupt queue
'''

class PAA5100JE():
    def __init__(self,
                 SPI_type: str,
                 reset: int = 0,
                 cs_pin: int = 0,
                 sck_pin: int = 0,
                 mosi_pin: int = 0,
                 miso_pin: int = 0):
        
        self.spi = SPI(SPI_type, baudrate=9600, polarity=0, phase=0, bits=16, firstbit=SPI.MSB,
                       sck=Pin(sck_pin, mode=0, pull=1), mosi=Pin(mosi_pin, mode=0, pull=1),
                       miso=Pin(miso_pin, mode=1))
        
        self.select = Pin(cs_pin, Pin.OUT)
        self.reset = Pin(reset, Pin.IN)
        
        self.select.value(0)
        self.reset.value(0)
        time.sleep_ms(50)
        self.select.value(1)
        self.reset.value(1)
        
        time.sleep_ms(50) # Motion delay after reset

        self.flo = BreakoutPAA5100(SPI_type, cs_pin, sck_pin, mosi_pin, miso_pin)
        self.flo.set_rotation(BreakoutPAA5100.DEGREES_0)
        self.x = 0
        self.y = 0
    
    def read_register(self, addrs: int):
        """
        addrs < 128
        """
        # ensure MSB=0
        addrs = addrs & 0x7f
        addrs = addrs.to_bytes(1, 'little')
        self.select.value(0)
        self.spi.write(addrs)
        time.sleep_us(100)  # tSRAD
        data = self.spi.read(1)
        time.sleep_us(1)  # tSCLK-NCS for read operation is 120ns
        self.select.value(1)
        time.sleep_us(19)  # tSRW/tSRR (=20us) minus tSCLK-NCS
        return data
        
    def send_values(self, values):
        self.select.value(0)  # Select the device
        for value in values:
            self.spi.write(value.to_bytes(2, 'big'))  # Sending each value as 2 bytes
        self.select.value(1)  # Deselect the device

    def read_motion(self):
        # Read x and y coordinates from PAA5100JE sensor
        self.select.value(0)
        delta = self.flo.get_motion()
        if delta is not None:
            self.x, self.y = delta
        else:
            self.x, self.y = 0, 0
        return self.x, self.y
        self.select.value(1)
    
    def config(self):
        ID = int.from_bytes(self.read_register(0x2a), 'little')
        assert ID == 0x04, "bad SROM v={}".format(ID)
        self.select.value(1)
        time.sleep_ms(10)
        
    def shut_down(self, deinitSPI=True):
        self.select.value(0) # Select the device
        time.sleep_ms(1)
        self.reset.value(1)
        time.sleep_ms(1)
        self.select.value(1) # Deselect the device
        time.sleep_ms(1)
        if deinitSPI:
            self.spi.deinit()
        
class MotionDetector(Analog_input):
    def __init__(self, reset, cs1, cs2,
                 name='MotDet', threshold=1, calib_coef=1,
                 sampling_rate=100, event='motion'):
        
        # Create SPI objects (SPI_type, sck, mosi, miso, cs)
        # change to same SPI because same sck used (and change chip select)
        self.motSen_x = PAA5100JE(0, reset, cs1, 18, 19, 16)
        self.motSen_y = PAA5100JE(1, reset, cs2, 10, 11, 12)
        
        motSen_x.config()
        motSen_y.config()
        self.threshold = threshold
        self.calib_coef = calib_coef
        
        self.prev_x = 0
        self.prev_y = 0        
        self.delta_x, self.delta_y = 0, 0    # accumulated position
        self._delta_x, self._delta_y = 0, 0  # instantaneous position
        self.x, self.y = 0, 0  # to be accessed from the task, unit=mm
        
        # Parent
        Analog_input.__init__(self, pin=None, name=name + '-X', sampling_rate=int(sampling_rate),
                              threshold=threshold, rising_event=event, falling_event=None,
                              data_type='l')
        self.data_chx = self.data_channel
        self.data_chy = Data_channel(name + '-Y', sampling_rate, data_type='l')
        self.crossing_direction = True  # to conform to the Analog_input syntax
        self.timestamp = fw.current_time
        self.acquiring = False
        
        gc.collect()
        time.sleep_ms(2)
      
    def reset_delta(self):
        # reset the accumulated position data
        self.delta_x, self.delta_y = 0, 0
        
    def read_sample(self):
        # Units are in millimeters
        current_motion_x = self.motSen_x.read_motion()
        current_motion_y = self.motSen_y.read_motion()
        
        if current_motion_x is not None and current_motion_y is not None:
            self.delta_x = current_motion_x[0]
            self.delta_y = current_motion_y[1]

            self._delta_x = self.delta_x - self.prev_x
            self._delta_y = self.delta_y - self.prev_y
            
            # Update previous coordinates
            self.prev_x = self.delta_x
            self.prev_y = self.delta_y
        
        disp = self.displacement()
        angle = self.tilt_angle()        
        print(f"x coordinate: {self.curr_x:>10.5f} | y coordinate: {self.curr_y:>10.5f} | Displacement: {disp:>12.5f} | Tilt angle: {angle:>11.5f}")
    
    def displacement(self):
        # Calculate displacement using the change of coordinates with time
        disp = math.sqrt(self._delta_x**2 + self._delta_y**2)
        return disp

    def tilt_angle(self):
        # Calculate tilt angle with respect to positive y axis (in degrees)
        if self.delta_y == 0:
            return 0
        angle = math.atan2(self.delta_x, self.delta_y) * 180 / math.pi
        return angle
    
    def _timer_ISR(self, t):
        "Read a sample to the buffer, update write index."
        self.read_sample()
        self.data_chx.put(self._delta_x)
        self.data_chy.put(self._delta_y)

        if self.delta_x**2 + self.delta_y**2 >= self.threshold:
            self.x = self.delta_x
            self.y = self.delta_y
            self.reset_delta()
            self.timestamp = fw.current_time
            interrupt_queue.put(self.ID)

    def _stop_acquisition(self):
        # Stop sampling analog input values.
        self.timer.deinit()
        self.data_chx.stop()
        self.data_chy.stop()
        self.motSen_x.shut_down(deinitSPI=False)      
        self.motSen_y.shut_down()
        self.acquiring = False
        self.reset_data()
        
    def _start_acquisition(self):
        # Start sampling analog input values.
        self.timer.init(freq=self.data_chx.sampling_rate)
        self.timer.callback(self._timer_ISR)
        self.acquiring = True

    def record(self):
        "Start streaming data to computer."
        self.data_chx.record()
        self.data_chy.record()
        if not self.acquiring:
            self._start_acquisition()
