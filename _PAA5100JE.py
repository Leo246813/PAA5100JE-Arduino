import time, math, gc
from array import array
from machine import Pin, SPI
from pyControl.hardware import *
from breakout_paa5100 import BreakoutPAA5100
'''
Changes: PAA5100JE does not have SROM ID, need to change way to interrupt queue
'''

class PAA5100JE():
    def __init__(self,
                 SPI_type: str,
                 reset: str = None,
                 cs: str = None,
                 sck: str = None,
                 mosi: str = None,
                 miso: str = None):
        
        # SPI_type = 'SPI1' or 'SPI2' or 'softSPI'
        SPIparams = {'baudrate': 9600, 'polarity': 0, 'phase': 1,
                     'bits': 8, 'firstbit': machine.SPI.MSB}
        if '1' in SPI_type:
            self.spi = machine.SPI(1, **SPIparams)

        elif '2' in SPI_type:
            self.spi = machine.SPI(2, **SPIparams)

        elif 'soft' in SPI_type.lower():  # Works for newer versions of micropython
            self.spi = machine.SoftSPI(sck=machine.Pin(sck, mode=machine.Pin.OUT, pull=machine.Pin.PULL_DOWN),
                                       mosi=machine.Pin(mosi, mode=machine.Pin.OUT, pull=machine.Pin.PULL_DOWN),
                                       miso=machine.Pin(miso, mode=machine.Pin.IN),
                                       **SPIparams
                                       )
        
        self.select = Digital_output(pin=CS, inverted=True)
        self.reset = Digital_output(pin=reset, inverted=True)
        
        self.reset.off()
        self.select.off()

        time.sleep_ms(50) # Motion delay after reset
        
        self.flo = BreakoutPAA5100(SPI_type, cs, sck, mosi, miso)
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
        self.select.on()
        self.spi.write(addrs)
        time.sleep_us(2)  # tSRAD
        data = self.spi.read(1)
        time.sleep_us(1)  # tSCLK-NCS for read operation is 120ns
        self.select.off()
        time.sleep_us(19)  # tSRW/tSRR (=20us) minus tSCLK-NCS
        return data
       
    def write_register(self, addrs: int, data: int):
        """
        addrs < 128
        """
        # flip the MSB to 1:
        addrs = addrs | 0x80
        addrs = addrs.to_bytes(1, 'little')
        data = data.to_bytes(1, 'little')
        self.select.on()
        self.spi.write(addrs)
        self.spi.write(data)
        time.sleep_us(20)  # tSCLK-NCS for write operation
        self.select.off()
        time.sleep_us(100)  # tSWW/tSWR (=120us) minus tSCLK-NCS. Could be shortened, but is looks like a safe lower bound
   
    def send_values(self, values):
        self.select.on()  # Select the device
        for value in values:
            self.spi.write(value.to_bytes(2, 'big'))  # Sending each value as 2 bytes
        self.select.off()  # Deselect the device

    def read_motion(self):
        # Read x and y coordinates from PAA5100JE sensor
        self.select.on()
        delta = self.flo.get_motion()
        if delta is not None:
            self.x, self.y = delta
        else:
            self.x, self.y = 0, 0
        return self.x, self.y
        self.select.off()
    
#     def config(self):
#         # Need ID for interrupting queue when sending data to computer
#         self.select.value(0)
#         ID = self.flo.get_id()
#         time.sleep_us(50)
#         self.select.value(1)
#         time.sleep_ms(1)
        
    def shut_down(self, deinitSPI=True):
        self.select.off()
        time.sleep_ms(1)
        self.select.on()
        self.reset.off()
        time.sleep_ms(60)
        self.read_motion()
        self.select.off()
        time.sleep_ms(1)
        if deinitSPI:
            self.spi.deinit()
        
class MotionDetector():
    def __init__(self, reset, cs1, cs2,
                 name='MotDet', threshold=1, calib_coef=1,
                 sampling_rate=100, event='motion'):
        
        # Create SPI objects
        self.motSen_x = PAA5100JE(SPI_type='SPI2', reset=reset, cs1)
        self.motSen_y = PAA5100JE(SPI_type='SPI2', reset=reset, cs2)
        
        self.motSen_x.config()
        self.motSen_y.config()
        print(f"Sensor X ID: {self.motSen_x.config()} | Sensor Y ID: {self.motSen_y.config()}")

        self._threshold = threshold
        self.calib_coef = calib_coef
        
        # Motion sensor variables
        self.x_buffer = array('i', [0, 0])
        self.y_buffer = array('i', [0, 0])
        self.x_buffer_mv = memoryview(self.x_buffer)
        self.y_buffer_mv = memoryview(self.y_buffer)
        
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
    
    @property
    def threshold(self):
        # return the value in cms
        return math.sqrt(self._threshold / 10)
    
    @threshold.setter
    def threshold(self, new_threshold):
        self._threshold = int((new_threshold)**2) * self.calib_coef
        self.reset_delta()
    
    def reset_delta(self):
        # reset the accumulated position data
        self.delta_x, self.delta_y = 0, 0
    
    def read_sample(self):
        # Units are in millimeters
        current_motion_x = self.motSen_x.read_motion()
        current_motion_y = self.motSen_y.read_motion()
        
        if current_motion_x is not None and current_motion_y is not None:
            self.x_buffer_mv = current_motion_x
            self.y_buffer_mv = current_motion_y

            # Corresponding current x and y coordinates from each sensor
            self.delta_x = self.x_buffer_mv[0]
            self.delta_y = self.y_buffer_mv[1]
            
            # Calculate change in coordinates
            self._delta_x = self.delta_x - self.prev_x
            self._delta_y = self.delta_y - self.prev_y
            
            # Update previous coordinates
            self.prev_x = self.delta_x
            self.prev_y = self.delta_y
        
        disp = self.displacement()
        angle = self.tilt_angle()
        
        if self.delta_x**2 + self.delta_y**2 >= self._threshold:
            print(f"x coordinate: {self.delta_x:>10.5f} | y coordinate: {self.delta_y:>10.5f} | Displacement: {disp:>12.5f} | Tilt angle: {angle:>11.5f}")
                
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
        # Read a sample to the buffer, update write index.
        self.read_sample()
        self.data_chx.put(self._delta_x)
        self.data_chy.put(self._delta_y)

        if self.delta_x**2 + self.delta_y**2 >= self._threshold:
            self.x = self.delta_x
            self.y = self.delta_y
            self.reset_delta()
            self.timestamp = fw.current_time
            interrupt_queue.put(self.ID) ### no self.ID (use timer/GPIO interrupt pin)

    def _stop_acquisition(self):
        # Stop sampling analog input values.
        self.timer.deinit()
        self.data_chx.stop()
        self.data_chy.stop()
        self.motSen_x.shut_down(deinitSPI=False)      
        self.motSen_y.shut_down()
        self.acquiring = False
        self.reset_delta()
        
    def _start_acquisition(self):
        # Start sampling analog input values.
        self.timer.init(freq=self.data_chx.sampling_rate)
        self.timer.callback(self._timer_ISR)
        self.acquiring = True

    def record(self):
        # Start streaming data to computer.
        self.data_chx.record()
        self.data_chy.record()
        if not self.acquiring:
            self._start_acquisition()

# --------------------------------------------------------
if __name__ == "__main__":    
    motion_sensor = MotionDetector(2, 17, 9)
    while True:
        motion_sensor.read_sample()
