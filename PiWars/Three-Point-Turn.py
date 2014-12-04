#!/usr/bin/python

# 'Three Point Turn' for KEITH at PiWars
# Created by Harry Merckel

import RPi.GPIO as GPIO
import time
import sys
import numpy
import smbus
import subprocess
from threading import Thread

# Setting up the GPIO pins for everything.
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
GPIO.setup(21, GPIO.OUT) # Motor 1 - forwards
GPIO.setup(19, GPIO.OUT) # Motor 1 - backwards
GPIO.setup(26, GPIO.OUT) # Motor 2 - forwards
GPIO.setup(24, GPIO.OUT) # Motor 2 - backwards
GPIO.setup(11, GPIO.IN) #
GPIO.setup(12, GPIO.IN) #
GPIO.setup(13, GPIO.IN) # 
GPIO.setup(7, GPIO.IN) # Override switch
GPIO.setup(18, GPIO.IN) # Wheel encoder 1
GPIO.setup(22, GPIO.IN) # Wheel encoder 2

bus = smbus.SMBus(0) # Rev 1 Pi uses 0
#bus = smbus.SMBus(1) # Rev 2 Pi uses 1

# This section defines what to do to control the LEDs, with a fix for the occasional disconnect error.
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

# Setting motor PWM pins and speeds
m1f = GPIO.PWM(21, 50)
m2f = GPIO.PWM(26, 50)
m1b = GPIO.PWM(19, 50)
m2b = GPIO.PWM(24, 50)
m1b.start(0)
m2b.start(0)
m1f.start(0)
m2f.start(0)

def motor (l, r):
    if l > 100 or r > 100:
        return
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
motor(0,0)

def top():
    while True:
        mcpwritea(2**4)
        time.sleep(0.05)
        mcpwritea(0)
        time.sleep(0.03)
        mcpwritea(2**4)
        time.sleep(0.05)
        mcpwritea(0)
        time.sleep(0.03)
        mcpwritea(2**3)
        time.sleep(0.05)
        mcpwritea(0)
        time.sleep(0.03)
        mcpwritea(2**3)
        time.sleep(0.05)
        mcpwritea(0)
        time.sleep(0.03)
        
light = Thread(target=top)
light.setDaemon(True)
light.start()

def checkTicks():
    left = 100
    right = 97
    leftTicks = 0
    previousLeft = 0
    distance = 0
    target = 1410
    # This is the circumference of the wheel divided by the number of ticks per revolution, corrected for real world results.
    #  Circumference = 116.24mm
    #  Number of ticks = 16
    # Don't ask me why the motors act differently going forwards and backwards...
    posdistancePerTick = 3.55
    negdistancePerTick = 3.95
    # All of this is just setting the distance, going that distance, stopping and continuing
    #  with the next task. The turning is as fine tuned as we could get it, and it was all
    #  tested on the kitchen floor!
    while distance < target:
        motor(left,right)
        leftStatus = GPIO.input(22)
        if previousLeft == 0:
            if leftStatus == 1:
                previousLeft = 1
                leftTicks += 1
        if previousLeft == 1:
            if leftStatus == 0:
                previousLeft = 0
                leftTicks += 1
        time.sleep(0.005)
        distance = leftTicks * posdistancePerTick
    print leftTicks
    print distance
    leftTicks = 0
    distance = 0
    motor(0,0)
    time.sleep(0.3)
    motor(-59,59)
    #mcpwriteb(2)
    time.sleep(1)
    #mcpwriteb(0)
    motor(0,0)
    time.sleep(0.3)
    target = 650
    while distance < target:
        motor(left,right)
        leftStatus = GPIO.input(22)
        if previousLeft == 0:
            if leftStatus == 1:
                previousLeft = 1
                leftTicks += 1
        if previousLeft == 1:
            if leftStatus == 0:
                previousLeft = 0
                leftTicks += 1
        time.sleep(0.005)
        distance = leftTicks * posdistancePerTick
    print leftTicks
    print distance
    leftTicks = 0
    distance = 0
    motor(0,0)
    time.sleep(0.3)
    target = 1350
    while distance < target:
        motor(-100,-100)
        leftStatus = GPIO.input(22)
        if previousLeft == 0:
            if leftStatus == 1:
                previousLeft = 1
                leftTicks += 1
        if previousLeft == 1:
            if leftStatus == 0:
                previousLeft = 0
                leftTicks += 1
        time.sleep(0.005)
        distance = leftTicks * negdistancePerTick
    print leftTicks
    print distance
    leftTicks = 0
    distance = 0
    motor(0,0)
    time.sleep(0.3)
    target = 600
    while distance < target:
        motor(left,right)
        leftStatus = GPIO.input(22)
        if previousLeft == 0:
            if leftStatus == 1:
                previousLeft = 1
                leftTicks += 1
        if previousLeft == 1:
            if leftStatus == 0:
                previousLeft = 0
                leftTicks += 1
        time.sleep(0.005)
        distance = leftTicks * posdistancePerTick
    print leftTicks
    print distance
    leftTicks = 0
    distance = 0
    motor(0,0)
    time.sleep(0.3)
    motor(-60,60)
    #mcpwriteb(2)
    time.sleep(1)
    #mcpwriteb(0)
    motor(0,0)
    target = 1450
    time.sleep(0.3)
    while distance < target:
        motor(left,right)
        leftStatus = GPIO.input(22)
        if previousLeft == 0:
            if leftStatus == 1:
                previousLeft = 1
                leftTicks += 1
        if previousLeft == 1:
            if leftStatus == 0:
                previousLeft = 0
                leftTicks += 1
        time.sleep(0.005)
        distance = leftTicks * posdistancePerTick
    print leftTicks
    print distance
    leftTicks = 0
    distance = 0
    motor(0,0)
    #motor(0,100)
    #time.sleep(1.05)
        
    motor(0,0)
    leftTicks = 0
    distance = 0
        
checkTicks()
mcpwritea(0)
mcpwriteb(0)
motor(0,0)
GPIO.cleanup()
                