#!/usr/bin/python

# 'Proximity Alert' for KEITH at PiWars

# Created by Harry Merckel
# Uses edited and improved code from https://github.com/chrisalexander/initio-pirocon-test/blob/master/sonar.py

import RPi.GPIO as GPIO
import time
import numpy
import random
import sys
import smbus

mcp = smbus.SMBus(0)  # Rev 1 RasPi uses 0
#mcp = smbus.SMBus(1) # Rev 2 RasPi uses 1

# This section defines what to do to control the LEDs, with a fix for the occasional disconnect error.
def mcpwritea(num):
    try:
        mcp.write_byte_data(0x20, 0x14, num)
    # Sometimes the smbus module fails to send instructions and returns an IOError, which is fixed by running i2cdetect which rescans the I2C bus.
    except IOError:
        subprocess.call(['i2cdetect', '-y', '0'])
def mcpwriteb(num):
    try:
        mcp.write_byte_data(0x20, 0x15, num)
    except IOError:
        subprocess.call(['i2cdetect', '-y', '0'])

# Set all MCP23017 pins as outputs.
mcp.write_byte_data(0x20,0x00,0x00)
mcp.write_byte_data(0x20,0x01,0x00)

# Set output all output bits to 0 (Just in case they weren't cleared before).
mcpwritea(0)
mcpwriteb(0)

# Setting up the GPIO pins for the motors and override switch
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
GPIO.setup(21, GPIO.OUT) # Motor 1 - forwards
GPIO.setup(19, GPIO.OUT) # Motor 1 - backwards
GPIO.setup(26, GPIO.OUT) # Motor 2 - forwards
GPIO.setup(24, GPIO.OUT) # Motor 2 - backwards
GPIO.setup(7, GPIO.IN, pull_up_down=GPIO.PUD_UP) # Override switch

# Setting up PWM for speed control and starting motors at 0 so no output.
m1f = GPIO.PWM(21, 50)
m1b = GPIO.PWM(19, 50)
m2f = GPIO.PWM(26, 50)
m2b = GPIO.PWM(24, 50)

m1f.start(0)
m1b.start(0)
m2f.start(0)
m2b.start(0)

# I changed the pin used on the PiRoCon for the ultrasonic distance sensor,
#  so I left this as an easy to change variable
pin = 8

def query():
    GPIO.setup(pin, GPIO.OUT)
    ti = time.time()
       
    distlist = [0.0] * 7
    ts=time.time()
    # This takes 7 very quick readings from the ultrasonic sensor. It takes the median of the 7 readings to get rid of bad results.
    for k in range(7):
        # Output a very short pulse as a ping to the sensor, then switches to input waiting for a response
        GPIO.output(pin, 1)
        time.sleep(0.00001)
        GPIO.output(pin, 0)
        time0=time.time()
        GPIO.setup(pin, GPIO.IN)
        
        # Waiting for return to start
        time1=time0
        while ((GPIO.input(pin)==0) and ((time1-time0) < 0.02)):
            time1=time.time()

        time1=time.time()
        time2=time1

        # Timing and waiting for return to finish
        while ((GPIO.input(pin)==1) and ((time2-time1) < 0.02)):
            time2=time.time()
        time2=time.time()

        time3=(time2-time1)

        # Calculation of distance -> return time * speed of sound / 2 (ping has to go twice distance) * 100 (to convert m to cm)
        distance=time3*343/2*100
        distlist[k]=distance

        GPIO.setup(pin, GPIO.OUT)
        tf = time.time() - ts
        time.sleep(0.01)
    # Taking the median of the 7 results
    distance = (sorted(distlist))[4]
    
    print distance
    GPIO.cleanup(pin)
    return distance
    
# This function is to simplify the motor control instead of having to use the ChangeDutyCycle function throughout the program.
# l is the left motor. r is the right motor.
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
        
# Checking the distance detected by the ultrasonic sensor and acting based upon it
def check():
    global turns
    min = 4
    slow = 15
    distance = query()
	
    # Checking the distance and slowing down as KEITH gets closer to the wall. Front LED array counts distance down - in binary...
    mcpwriteb(int(distance)-3)
    if distance > slow:
        motor(50,50)
        return
    if distance < slow and distance > min:
        motor(20,20)
        return
    if distance < min:
        motor(0,0)
        print "Stopped"
        # The routine to flash the orange LEDs on top.
		mcpwriteb(0)
        reps = 10
        for i in range(0,reps):
            mcpwritea(2**4)
            time.sleep(0.05)
            mcpwritea(0)
            time.sleep(0.02)
            mcpwritea(2**4)
            time.sleep(0.05)
            mcpwritea(0)
            time.sleep(0.02)
            mcpwritea(2**3)
            time.sleep(0.05)
            mcpwritea(0)
            time.sleep(0.02)
            mcpwritea(2**3)
            time.sleep(0.05)
            mcpwritea(0)
            time.sleep(0.02)
        GPIO.cleanup()
        sys.exit()
        return
            
def flash():
    num = int(distance)
    mcwriteb(num)

# Checks if a connection is not made on pin 7 (switch) as an emergency stop and restarts program.
def checkSwitch():
    active = not GPIO.input(7)
    if active:
        return
    else:
        motor(0, 0)
        start()

# The start function checks for the switch every two seconds if it's off.
def start():
    while True:
        print "start"
        active = not GPIO.input(7)
        if active:
            return
        else:
            time.sleep(2)

motor(0, 0)

start()

# This loops runs it all, keeping the robot at max speed unless it encounters an obstacle
while True:
    checkSwitch()
    check()
    time.sleep(0.02)

motor (0, 0)

GPIO.cleanup()