import smbus        #import SMBus module of I2C
from time import sleep  #import sleep
import time
import math
import wiringpi
import RPi.GPIO as GPIO




class pan_tilt:

    #some MPU6050 Registers and their Address
    Register_A     = 0              #Address of Configuration register A
    Register_B     = 0x01           #Address of configuration register B
    Register_mode  = 0x02           #Address of mode register

    X_axis_H    = 0x03              #Address of X-axis MSB data register
    Z_axis_H    = 0x05              #Address of Z-axis MSB data register
    Y_axis_H    = 0x07              #Address of Y-axis MSB data register
    declination = -0.00669          #define declination angle of location where measurement going to be done
    pi          = 3.14159265359     #define pi value

    ############ STEPPER  ###############

    DIR = 20   # Direction GPIO Pin
    STEP = 21  # Step GPIO Pin
    CW = 1     # Clockwise Rotation
    CCW = 0    # Counterclockwise Rotation
    SPR = 200  # Steps per Revolution (360 / 7.5)

    #GPIO.cleanup()
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(DIR, GPIO.OUT)
    GPIO.setup(STEP, GPIO.OUT)
    GPIO.output(DIR, CW)

    MODE = (23, 24, 25)   # Microstep Resolution GPIO Pins
    GPIO.setup(MODE, GPIO.OUT)
    RESOLUTION = {'Full': (0, 0, 0),
                  'Half': (1, 0, 0),
                  '1/4': (0, 1, 0),
                  '1/8': (1, 1, 0),
                  '1/16': (0, 0, 1),
                  '1/32': (1, 0, 1)}

    Res_adj = {'Full': 1,'Half': 2,'1/4': 4,'1/8': 8,'1/16': 16,'1/32': 32}

    
    STEP_POSITION = 0
    

    """docstring for ClassName"""
    def __init__(self,bus = None, step = 'Full',flat_declination=138,delay_period=0.005):
        

        self.stepper_pos = 0
        self.step = step    
        GPIO.output(self.MODE, self.RESOLUTION[step])
        self.step_count = self.SPR * self.Res_adj[step]
        self.delay = .0208 / (self.Res_adj[step]*12)
        #self.bus = smbus.SMBus(1)    # or bus = smbus.SMBus(0) for older version boards
        self.bus = bus
        self.Device_Address = 0x1e   # HMC5883L magnetometer device address
        self.Magnetometer_Init(bus)     # initialize HMC5883L magnetometer 
        self.delay_period = delay_period
        self.flat_declination = flat_declination

        # use 'GPIO naming'
        wiringpi.wiringPiSetupGpio()
         
        # set #18 to be a PWM output
        wiringpi.pinMode(18, wiringpi.GPIO.PWM_OUTPUT)
         
        # set the PWM mode to milliseconds stype
        wiringpi.pwmSetMode(wiringpi.GPIO.PWM_MODE_MS)
         
        # divide down clock
        wiringpi.pwmSetClock(192)
        wiringpi.pwmSetRange(2000)
         
        

        ####
        wiringpi.pwmWrite(18,self.flat_declination)
        heading_angle = self.get_heading()      

        #GPIO.output(self.MODE, RESOLUTION[step])

    def stepper_get_current_pos(self):
        return self.STEP_POSITION

    def go_to_step(self,wanted_pos):
        case_CW =self.STEP_POSITION -  wanted_pos
        case_CCW = 200 - self.STEP_POSITION+wanted_pos
        print("case CW: ", case_CW, "case CCW: ", case_CCW ,"   CURRENT : ", self.STEP_POSITION  )
        if  case_CW < case_CCW:
            self.stepper_move_cw(delay=0.002,step_count=case_CW)
        else:
            self.stepper_move_ccw(delay=0.002,step_count=case_CCW)
    def Magnetometer_Init(self,bus):
            #write to Configuration Register A
            bus.write_byte_data(self.Device_Address, self.Register_A, 0x70)

            #Write to Configuration Register B for gain
            bus.write_byte_data(self.Device_Address, self.Register_B, 0xa0)

            #Write to mode Register for selecting mode
            bus.write_byte_data(self.Device_Address, self.Register_mode, 0)
        
        
    def read_raw_data(self,addr):
        
            #Read raw 16-bit value
            high = self.bus.read_byte_data(self.Device_Address, addr)
            low = self.bus.read_byte_data(self.Device_Address, addr+1)

            #concatenate higher and lower value
            value = ((high << 8) | low)

            #to get signed value from module
            if(value > 32768):
                value = value - 65536
            return value


    def get_heading(self,print_out = True):
            #Read Accelerometer raw value
            x = self.read_raw_data(self.X_axis_H)
            z = self.read_raw_data(self.Z_axis_H)
            y = self.read_raw_data(self.Y_axis_H)

            heading = math.atan2(y, x) + self.declination
            
            #Due to declination check for >360 degree
            if(heading > 2*self.pi):
                    heading = heading - 2*self.pi

            #check for sign
            if(heading < 0):
                    heading = heading + 2*self.pi

            #convert into angle
            heading_angle = int(heading * 180/self.pi)
            print("x: ",x,",y: ",y,",z: ",z)
            return heading_angle

    def calibrate_declination(self):
        delay_period = 0.005

        pulse_start = 60;

        while True:

            z = - self.read_raw_data(Z_axis_H)
            if z > 220:
                pulse_start += 1
                wiringpi.pwmWrite(18,pulse_start)
                time.sleep(delay_period)
            elif z < 224:
                pulse_start -= 1
                wiringpi.pwmWrite(18,pulse_start)
                time.sleep(delay_period)
            else:
                return True


    def stepper_move_cw(self,delay = 0.002,step_count = 100):

        GPIO.output(self.DIR, self.CW)
        for x in range(step_count):
            GPIO.output(self.STEP, GPIO.HIGH)
            sleep(self.delay)
            GPIO.output(self.STEP, GPIO.LOW)
            sleep(self.delay)

        self.STEP_POSITION = (self.STEP_POSITION + step_count) % 200

    def stepper_move_ccw(self,delay = 0.002,step_count = 100):

        GPIO.output(self.DIR, self.CCW)
        for x in range(step_count):
            GPIO.output(self.STEP, GPIO.HIGH)
            sleep(self.delay)
            GPIO.output(self.STEP, GPIO.LOW)
            sleep(self.delay)
        self.STEP_POSITION = (self.STEP_POSITION - step_count) % 200

    def servo_set_pos(self,pos):
        wiringpi.pwmWrite(18, pos)