'''
    Find Heading by using HMC5883L interface with Raspberry Pi using Python
	http://www.electronicwings.com
'''
#from pololu_drv8835_rpi import motors, MAX_SPEED

#import RPi.GPIO as GPIO

import pynmea2
import random
import serial
import logging
from pan_tilt_libs import *
import dash_daq as daq
import dash
from dash.dependencies import Input, Output
import dash_html_components as html


port = "/dev/ttyAMA0" 

#create a serial object
#ser = serial.Serial(port, baudrate = 9600, timeout = 0.5)


############ STEPPER  ###############

DIR = 20   # Direction GPIO Pin
STEP = 21  # Step GPIO Pin
CW = 1     # Clockwise Rotation
CCW = 0    # Counterclockwise Rotation
SPR = 200   # Steps per Revolution (360 / 7.5)

#GPIO.cleanup()
#GPIO.setmode(GPIO.BCM)
#GPIO.setup(DIR, GPIO.OUT)
#GPIO.setup(STEP, GPIO.OUT)
#GPIO.output(DIR, CW)

#MODE = (23, 24, 25)   # Microstep Resolution GPIO Pins
#GPIO.setup(MODE, GPIO.OUT)



def get_pan():
    pass

def get_til():
    pass 


print (" Reading Heading Angle")

ser = serial.Serial("/dev/ttyS0", 9600,timeout = 1)
bus = smbus.SMBus(1)    # or bus = smbus.SMBus(0) for older version boards
pt = pan_tilt(step = 'Full',bus = bus)
#calibrate_declination()

"""
pt = pan_tilt(step = 'Full',bus = bus)
try:
    while True: 
        try:
            data = ser.readline()
            print(data) 
        except Exception as e:
            raise
        else:
            pass
        finally:
            pass   
        #print ("Heading Angle = %d°" % pt.get_heading())
        sleep(1)
        
        #for pulse in range(60, 220, 1):
        #        wiringpi.pwmWrite(18, pulse)
        #        time.sleep(delay_period)
        #for pulse in range(220, 60, -1):
        #        wiringpi.pwmWrite(18, pulse)
        #        time.sleep(delay_period)

        #pt.servo_set_pos(100)
        pt.servo_set_pos(pos=random.randint(50,200))

        pt.stepper_move_cw(delay = 0.002)

        sleep(.5)
        print ("Heading Angle = %d°" % pt.get_heading())

        pt.stepper_move_ccw(delay = 0.0015)
        print ("Heading Angle = %d°" % pt.get_heading())
finally:
    print("clean up") 
    GPIO.cleanup() # cleanup all GPIO 

"""

app = dash.Dash(__name__)

theme = {
    'dark': False,
    'detail': '#007439',
    'primary': '#00EA64',
    'secondary': '#6E6E6E'
}

app.layout = html.Div(id='dark-theme-provider-demo', children=[
    html.Br(),
    daq.ToggleSwitch(
        id='daq-light-dark-theme',
        label=['Light', 'Dark'],
        style={'width': '250px', 'margin': 'auto'},
        value=False
    ),
    html.Div(
        id='dark-theme-component-demo',
        children=[
            daq.DarkThemeProvider(theme=theme, children=[
                daq.Knob( id='servo-knob',value=100,max= 250,min = 50),
                daq.Knob( id='stepper-knob', value=1, max=200, min=0),
                daq.Slider( 'step-slider',min=1,max=100,value=1,handleLabel={"showCurrentValue": True, "label": "VALUE"},step=1),
                daq.StopButton(buttonText='CW',id='step_cw',label='Default',n_clicks=0),
                daq.StopButton(buttonText='CCW',id='step_ccw',label='Default',n_clicks=0),

        ])
        ],
        style={'display': 'block', 'margin-left': 'calc(50% - 110px)'},

    ),
html.Div([], id='value-container', style={'display': 'none'}),
html.Div([], id='value-container2', style={'display': 'none'}),
html.Div([], id='value-container3', style={'display': 'none'}),
html.Div([], id='value-container4', style={'display': 'none'}),

])

#pt.servo_set_pos(100)

@app.callback(
    Output('dark-theme-component-demo', 'children'),
    [Input('daq-light-dark-theme', 'value')]
)
def turn_dark(dark_theme):
    if(dark_theme):
        theme.update(
            dark=True
        )
    else:
        theme.update(
            dark=False
        )
    return daq.DarkThemeProvider(theme=theme, children=[
                                 daq.Knob( id='servo-knob',value=138,max= 250,min = 50),
                                daq.Knob( id='stepper-knob', value=1, max=200, min=0),
        daq.Slider('step-slider', min=1, max=100, value=1, handleLabel={"showCurrentValue": True, "label": "VALUE"},
                   step=1),
                                daq.StopButton(buttonText='CW',id='step_cw',label='Default',n_clicks=0),
                                daq.StopButton(buttonText='CCW',id='step_ccw',label='Default',n_clicks=0),

    ] )


@app.callback(
    Output('dark-theme-provider-demo', 'style'),
    [Input('daq-light-dark-theme', 'value')]
)
def change_bg(dark_theme):
    if(dark_theme):
        return {'background-color': '#303030', 'color': 'white'}
    else:
        return {'background-color': 'white', 'color': 'black'}

@app.callback(
    Output('value-container', 'children'),
    [Input('servo-knob', 'value')])
def update_output(value):
    print('update_output',value)
    pt.servo_set_pos(int(value))
    return 0

@app.callback(
    Output('value-container2', 'children'),
    [Input('stepper-knob', 'value')])
def update_stepper_output(value):
    print('update_stepper_output',value)
    pt.go_to_step(int(value))
    #pt.stepper_move_ccw()
    return 0

@app.callback(
    dash.dependencies.Output('value-container3', 'children'),
    [dash.dependencies.Input('step_cw', 'n_clicks')],
    [dash.dependencies.State('step-slider', 'value'),])
def update_output(n_clicks,value):
    pt.stepper_move_cw(step_count=value,delay=0.001)
    return 'The stop button has been clicked {} times.'.format(n_clicks)

@app.callback(
    dash.dependencies.Output('value-container4', 'children'),
    [dash.dependencies.Input('step_ccw', 'n_clicks')],
    [dash.dependencies.State('step-slider', 'value'),])
def update_output(n_clicks,value):
    pt.stepper_move_ccw(step_count=value,delay=0.001)
    return 'The stop button has been clicked {} times.'.format(n_clicks)

import atexit
def all_done():
    print("Cleaning...")
    GPIO.cleanup()
    print("Done...")

if __name__ == '__main__':
    atexit.register(all_done)
    app.suppress_callback_exceptions=True
    app.run_server(debug=True, host='0.0.0.0' )