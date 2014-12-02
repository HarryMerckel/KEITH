#!/usr/bin/python

# 'Line Follow' for KEITH at PiWars
# Created by Harry Merckel

# Uses edited and improved code from https://github.com/chrisalexander/initio-pirocon-test/blob/master/sonar.py

import RPi.GPIO as GPIO
import time
import numpy
import smbus
import subprocess
import sys
from threading import Thread

try:
    # Setting up the GPIO pins for the motors and override switch
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(21, GPIO.OUT) # Motor 1 - forwards
    GPIO.setup(19, GPIO.OUT) # Motor 1 - backwards
    GPIO.setup(26, GPIO.OUT) # Motor 2 - forwards
    GPIO.setup(24, GPIO.OUT) # Motor 2 - backwards
    GPIO.setup(11, GPIO.IN) # }
    GPIO.setup(12, GPIO.IN) # } Line Sensors
    GPIO.setup(13, GPIO.IN) # }
    GPIO.setup(7, GPIO.IN) # Override switch

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

    global l
    global c
    global r

    # Setting up PWM for speed control and starting motors at 0 so no output.
    m1f = GPIO.PWM(21, 50)
    m1b = GPIO.PWM(19, 50)
    m2f = GPIO.PWM(26, 50)
    m2b = GPIO.PWM(24, 50)

    m1f.start(0)
    m1b.start(0)
    m2f.start(0)
    m2b.start(0)

    # This function is to simplify the motor control instead of having to use the ChangeDutyCycle function throughout the program.
    def motor (left, right):
        if left > 0:
            m1f.ChangeDutyCycle(left)
            m1b.ChangeDutyCycle(0)
        else:
            m1f.ChangeDutyCycle(0)
            m1b.ChangeDutyCycle(-left)
        if right > 0:
            m2f.ChangeDutyCycle(right)
            m2b.ChangeDutyCycle(0)
        else:
            m2f.ChangeDutyCycle(0)
            m2b.ChangeDutyCycle(-right)
            
    def query():
        pin = 8
        GPIO.setup(pin, GPIO.OUT)
        global distance
        distlist = [0.0] * 7

        # This takes very quick readings from the ultrasonic sensor. Takes the rolling mean of 6 results to get rid of anomalous results.
        while True:
            # Output a very short pulse as a ping to the sensor, then switches to input waiting for a response
            GPIO.output(pin, 1)
            time.sleep(0.00001)
            GPIO.output(pin, 0)
            time0 = time.time()
            GPIO.setup(pin, GPIO.IN)

            # Waiting for return to start
            time1=time0
            while ((GPIO.input(pin) == 0) and ((time1 - time0) < 0.02)):
                time1 = time.time()

            time1 = time.time()
            time2 = time1

            # Timing and waiting for return to finish
            while ((GPIO.input(pin) == 1) and ((time2 - time1) < 0.02)):
                time2 = time.time()
            time2 = time.time()

            time3 = (time2-time1)

            # Calculation of distance -> return time * speed of sound (m/s) (approx. at 20 degrees C) / 2 (ping has to go to and from object) * 100 (m to cm)
            cDistance = time3*343/2*100

            GPIO.setup(pin, GPIO.OUT)
            # Removing the oldest datapoint in the list and adding the new one for an accurate reading as possible
            del distlist[0]
            distlist.append(cDistance)
            # Taking the mean of the 10 rolling results
            distance = numpy.mean(distlist)
            #print distance
            time.sleep(0.02)

    def lights():
        global stopped
        stopped = 0
        while True:
            if stopped == 0:
                # Flashy top lights plus constant headlights and underlights
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
                continue
            else:
                time.sleep(0.2)

    def lineCheck():
        while True:
            # A very fast loop to gain the most up-to-date results from the sensors as possible.
            global l
            global c
            global r
            l = GPIO.input(11)
            c = GPIO.input(12)
            r = GPIO.input(13)
            time.sleep(0.0025)
            
    def checkLine():
        global distance
        # Checks if the distance detected by the ultrasonic sensor is less than 10cm so we can catch it!
        if distance < 10:
            motor(0,0)
            return
        global l
        global c
        global r
        # This checks the line following sensor and acts accordingly.
        if not l and c and not r:
            motor(100,100)
            mcpwriteb(0)
            return
        if l and c and not r:
            motor(-100,100)
            mcpwriteb(1)
            return
        if l and not c and not r:
            motor(-70,100)
            mcpwriteb(1)
            return
        if not l and c and r:
            motor(100,-100)
            mcpwriteb(128)
            return
        if not l and not c and r:
            motor(100,-70)
            mcpwriteb(128)
            return
        if not l and not c and not r:
            motor(100,100)
            mcpwriteb(129)
            return
        if l and c and r:
            motor(0,0)
            mcpwriteb(129)
            return
                
    # Checks if a connection is made on pin 7 (switch) to stop and start the program.
    def checkSwitch():
        active = GPIO.input(7)
        if active:
            return
            stopped = 0
        else:
            motor(0, 0)
            start()
                
    # The start function checks for the switch every half a second if it's off.
    def start():
        while True:
            #print "start"
            mcpwriteb(0)
            global stopped
            stopped = 1
            active = GPIO.input(7)
            if active:
                stopped = 0
                return
            else:
                time.sleep(0.2)
                
    motor(0, 0)
    # Starting the line checker and distance checker as threads to keep them running alongside the rest of the code
    t = Thread(target=lineCheck)
    t.setDaemon(True)
    t.start()
    t = Thread(target=query)
    t.setDaemon(True)
    t.start()
    light = Thread(target=lights)
    light.setDaemon(True)
    light.start()
    # Small wait to let results start coming in
    time.sleep(1)

    start()

    # This loops runs it all.
    while True:
        checkSwitch()
        checkLine()
        time.sleep(0.005)
except:
    motor (0, 0)
    mcpwritea(0)
    mcpwriteb(0)
    GPIO.cleanup()
    sys.exit()
