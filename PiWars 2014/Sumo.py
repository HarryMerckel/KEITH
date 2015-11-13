#!/usr/bin/python

# 'Sumo' for KEITH at PiWars
# Created by Harry Merckel

# Uses basis of:
    # wii_remote_1.py
    # Connect a Nintendo Wii Remote via Bluetooth
    # and  read the button states in Python.
    # Project URL :
    # http://www.raspberrypi-spy.co.uk/?p=1101
    # Author : Matt Hawkins
    # Date   : 30/01/2013

# Uses heavily edited and improved code from https://github.com/chrisalexander/initio-pirocon-test/blob/master/sonar.py

# Import required Python libraries
import cwiid
import smbus
import time
import RPi.GPIO as GPIO
import numpy
import random
import sys
import subprocess
from threading import Thread

bus = smbus.SMBus(0) # Rev 1 Pi uses 0
#bus = smbus.SMBus(1) # Rev 2 Pi uses 1

def mcpwritea(num):
    try:
        bus.write_byte_data(0x20, 0x14, num)
    # Sometimes the smbus module fails to send instructions and returns an IOError, which is fixed by running i2cdetect which rescans the I2C bus.
    except IOError:
        subprocess.call(['i2cdetect', '-y', '0'])
def mcpwriteb(num):
    try:
        bus.write_byte_data(0x20, 0x15, num)
    except IOError:
        subprocess.call(['i2cdetect', '-y', '0'])

# Set all MCP23017 pins as outputs.
bus.write_byte_data(0x20,0x00,0x00)
bus.write_byte_data(0x20,0x01,0x00)

# Set output all output bits to 0 (Just in case they weren't cleared before).
mcpwritea(0)
mcpwriteb(0)

button_delay = 0.1

print 'Press 1 + 2 on your Wii Remote now ...'
mcpwriteb(2+2**6)
time.sleep(1)

# Connect to the Wii Remote. If it times out then try again until it does work. LEDs flash to indicate success or failure.

def connect():
    global wii
    # Connecting to the wiimote, lights to show status!
    while True:
        try:
            #print "Trying connection"
            mcpwriteb(2+2**6)
            wii=cwiid.Wiimote()
        except RuntimeError:
            #print "Error opening wiimote connection"
            mcpwriteb(1+2**7)
            time.sleep(0.5)
            mcpwriteb(0)
            time.sleep(0.5)
            mcpwriteb(1+2**7)
            time.sleep(0.5)
            mcpwriteb(0)
            time.sleep(0.5)
            mcpwriteb(1+2**7)
            time.sleep(0.5)
            mcpwriteb(0)
            time.sleep(0.5)
            mcpwriteb(1+2**7)
            time.sleep(0.5)
            mcpwriteb(0)
            continue
        else:
            #print "Connection made"
            mcpwriteb(2**2+2**5)
            time.sleep(0.5)
            mcpwriteb(0)
            time.sleep(0.5)
            mcpwriteb(2**2+2**5)
            time.sleep(0.5)
            mcpwriteb(0)
            return

connect()
            
print 'Wii Remote connected'

wii.rpt_mode = cwiid.RPT_BTN

# Setting up the GPIO pins for the motors and override switch
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
GPIO.setup(21, GPIO.OUT) # Motor 1 - forwards
GPIO.setup(19, GPIO.OUT) # Motor 1 - backwards
GPIO.setup(26, GPIO.OUT) # Motor 2 - forwards
GPIO.setup(24, GPIO.OUT) # Motor 2 - backwards
GPIO.setup(11, GPIO.IN) #
GPIO.setup(12, GPIO.IN) #
GPIO.setup(13, GPIO.IN) #
GPIO.setup(18, GPIO.IN) # Wheel encoder
GPIO.setup(22, GPIO.IN) # Wheel encoder
GPIO.setup(7, GPIO.IN, pull_up_down=GPIO.PUD_UP) # Override switch

servo = 16
GPIO.setup(servo, GPIO.OUT)
servoPWM = GPIO.PWM(servo, 100)
servoPWM.start(0)

# Setting up PWM for speed control and starting motors at 0 so no output. Pins are as default on the PiRoCon
#  board and 50Hz is fine for both the Pi and the control board.
m1f = GPIO.PWM(21, 50)
m1b = GPIO.PWM(19, 50)
m2f = GPIO.PWM(26, 50)
m2b = GPIO.PWM(24, 50)

m1f.start(0)
m1b.start(0)
m2f.start(0)
m2b.start(0)
    
# This function is to simplify the motor control instead of having to use the ChangeDutyCycle function throughout the program.
def motor (l, r):
    if l > 0:
        m1f.ChangeDutyCycle(l)
        m1b.ChangeDutyCycle(0)
    else:
        m1f.ChangeDutyCycle(0)
        m1b.ChangeDutyCycle(-l)
    if r > 0:
        m2f.ChangeDutyCycle(r)
        m2b.ChangeDutyCycle(0)
    else:
        m2f.ChangeDutyCycle(0)
        m2b.ChangeDutyCycle(-r)

# Checks if a connection is made on pin 7 (switch) as an emergency stop and restarts program.
def checkSwitch():
    active = True #not GPIO.input(7)
    if active:
        return
    else:
        motor(0, 0)
        start()

# The start function checks for the switch every two seconds if it's off.
def start():
    while True:
        active = True #not GPIO.input(7)
        if active:
            print "..."
            return
        else:
            time.sleep(2)

def lights():
    servopos = 1
    while True:
        buttons = wii.state['buttons']
        lightstatusa = 2**1
        lightstatusa += 2**0
        if (buttons & cwiid.BTN_A):
            lightstatusa += 2**7
        try:
            mcpwritea(lightstatusa)
        except IOError:
            subprocess.call(['i2cdetect', '-y', '0'])
        time.sleep(0.1)
        
light = Thread(target=lights)
light.setDaemon(True)
light.start()

def frontSweep():
    B = 0
    while True:
        buttons = wii.state['buttons']
        if (buttons & cwiid.BTN_B):
            if B == 0:
                B = 1
                print B
                time.sleep(0.5)
                continue
            if B == 1:
                B = 0
                print B
                time.sleep(0.5)
                continue
        if B == 1:
            for i in range(0,7,1):
                MyData = (2**i)
                mcpwriteb(MyData)
                time.sleep(0.05)
            for i in range(7,0,-1):
                MyData = (2**i)
                mcpwriteb(MyData)
                time.sleep(0.05)
        if B == 0:
            mcpwriteb(0)

sweep = Thread(target=frontSweep)
sweep.setDaemon(True)
sweep.start()
            
motor(0, 0)

global speed
speed = 0

start()

# And finally, the loop that runs it all!

while True:
    checkSwitch()
    motor(0,0)
    buttons = wii.state['buttons']
    # If Plus and Minus buttons pressed together then rumble and quit.
    if (buttons - cwiid.BTN_PLUS - cwiid.BTN_MINUS == 0):    
        print '\nClosing connection ...'
        wii.rumble = 1
        time.sleep(1)
        wii.rumble = 0
        exit(wii)    
    
    # First checking if the direction buttons and the movement buttons are pressed and acting
    #  accordingly, restarting to loop so nothing strange happens.
    # This is also checking the distance detected by the ultrasonic sensor and stopping forward
    #  movement if it is less than 15cm from an object.
    
    if (buttons & cwiid.BTN_RIGHT):
        speed = 100
        
    if (buttons & cwiid.BTN_LEFT):
        speed = 50
    
    if (buttons - cwiid.BTN_UP - cwiid.BTN_2 == 0):
        motor(0,speed)
        time.sleep(button_delay)
        continue

    if (buttons - cwiid.BTN_UP - cwiid.BTN_1 == 0):
        motor(0,0-speed)
        time.sleep(button_delay)
        continue
        
    if (buttons - cwiid.BTN_DOWN - cwiid.BTN_2 == 0):
        motor(speed,0)
        time.sleep(button_delay)
        continue

    if (buttons - cwiid.BTN_DOWN - cwiid.BTN_1 == 0):
        motor(0-speed,0)
        time.sleep(button_delay)
        continue

    # Check if other buttons are pressed by doing a bitwise AND of the buttons number
    # and the predefined constant for that button.
    if (buttons & cwiid.BTN_UP):
        # Up is actually Left when holing the WiiMote horizontally, so we turn KEITH left. Left = Down, Right = Up, Down = Right, Up = Left.
        motor(0-speed,speed)
        time.sleep(button_delay)
        continue
        
    if (buttons & cwiid.BTN_DOWN):
        motor(speed,0-speed)
        time.sleep(button_delay)
        continue
        
    if (buttons & cwiid.BTN_1):
        motor(0-speed,0-speed)
        time.sleep(button_delay)
        continue

    if (buttons & cwiid.BTN_2):
        motor(speed,speed)
        time.sleep(button_delay)
        continue

motor (0, 0)

GPIO.cleanup()