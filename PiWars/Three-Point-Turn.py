#!/usr/bin/python

import RPi.GPIO as GPIO
import time
import sys
import numpy
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

def checkTicks():
    # Setting up all the variables and constants needed
    # The base power levels to the motors in % - the master will stay constant but the slave will change
    leftTicks = 0
    previousLeft = 0
    distance = 0
    target = 1000
    # This is the circumference of the wheel divided by the number of ticks per revolution.
    #  Circumference = 116.24mm
    #  Number of ticks = 16
    # Don's ask me why the motors act differently going forwards and backwards...
    posdistancePerTick = 3.55
    negdistancePerTick = 3.95
    #while True:
    while distance < target:
        motor(100,100)
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
    time.sleep(0.5)
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
    #motor(0,100)
    #time.sleep(1.05)
        
    motor(0,0)
    leftTicks = 0
    distance = 0
        
checkTicks()
        
motor(0,0)
GPIO.cleanup()
                