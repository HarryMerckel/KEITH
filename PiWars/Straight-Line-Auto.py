#!/usr/bin/python

# 'Straight Line (Automatic)' for KEITH at PiWars
# Created by Harry Merckel

import RPi.GPIO as GPIO
import time
import sys
import numpy
import smbus
from threading import Thread

# Setting up the GPIO pins for everything.
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
GPIO.setup(21, GPIO.OUT) # Motor 1 - forwards
GPIO.setup(19, GPIO.OUT) # Motor 1 - backwards
GPIO.setup(26, GPIO.OUT) # Motor 2 - forwards
GPIO.setup(24, GPIO.OUT) # Motor 2 - backwards
GPIO.setup(7, GPIO.IN) # Override switch
GPIO.setup(18, GPIO.IN) # Wheel encoder 1
GPIO.setup(22, GPIO.IN) # Wheel encoder 2

# Setting motor PWM pins and speeds
m1f = GPIO.PWM(21, 50)
m2f = GPIO.PWM(26, 50)
m1b = GPIO.PWM(19, 50)
m2b = GPIO.PWM(24, 50)
m1b.start(0)
m2b.start(0)
m1f.start(0)
m2f.start(0)

bus = smbus.SMBus(0)  # Rev 1 Pi uses 0
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

def top():
    while True:
        mcpwritea(2**4)
        time.sleep(0.5)
        mcpwritea(2**3)
        time.sleep(0.5)
        
light = Thread(target=top)
light.setDaemon(True)
light.start()
        
# "Automatic, they said..."
# Slightly (not) corrected speeds for going in a straight(ish) line.
motor(100,100)
time.sleep(10)
mcpwritea(0)
GPIO.cleanup()

# Cut out PD code. Not used because the results were so variable that it was more accurate to just go at maximum speed.
# We think the problems are that there are too many variables with having tracks, cheap DC motors and unreliable wheel encoders!
"""
def checkTicks():
    # Setting up all the variables and constants needed
    # The base power levels to the motors in % - the master will stay constant but the slave will change
    master = 80
    slave = 79
    # PID constants
    Kp = 1.2
    Ki = 0
    Kd = 0.8
    Cp = 0
    Ci = 0
    Cd = 0
    # Error - the difference in number of ticks between the master and slave motors (left and right)
    error = 0
    previousError = 0
    previousLeft = 0
    previousRight = 0
    leftTicks = 0
    rightTicks = 0
    currentTime = time.time()
    previousTime = currentTime
    leftFive = [0.0] * 10
    rightFive = [0.0] * 10
    allLeft = []
    allRight = []
    #while True:
    for i in range(0,120):
        for i in range(0,50):
            leftStatus = GPIO.input(22)
            rightStatus = GPIO.input(18)
            if previousLeft == 0:
                if leftStatus == 1:
                    previousLeft = 1
                    leftTicks += 1
            if previousRight == 0:
                if rightStatus == 1:
                    previousRight = 1
                    rightTicks += 1
            if previousLeft == 1:
                if leftStatus == 0:
                    previousLeft = 0
                    leftTicks += 1
            if previousRight == 1:
                if rightStatus == 0:
                    previousRight = 0
                    rightTicks += 1
            time.sleep(0.005)
        leftFive.append(leftTicks)
        del leftFive[0]
        leftAvg = numpy.mean(leftFive)
        rightFive.append(rightTicks)
        del rightFive[0]
        rightAvg = numpy.mean(rightFive)
        allLeft.append(leftAvg)
        print leftAvg
        allRight.append(rightAvg)
        print rightAvg
        currentTime = time.time()
        deltaTime = currentTime - previousTime
        #print deltaTime
        error = leftAvg - rightAvg
        deltaError = error - previousError
        print deltaError
        Cp = Kp * error
        Ci += error * deltaTime
        if deltaTime > 0:
            Cd = deltaError / deltaTime
        else:
            Cd = 0
        previousTime = currentTime
        previousError = error
        PID = Cp + (Ki * Ci) + (Kd * Cd)
        #print str(PID)
        #if PID > 100: PID = 100
        slave += PID
        print slave
        motor(master,slave)
        leftTicks = 0
        rightTicks = 0
        
        
checkTicks()
"""