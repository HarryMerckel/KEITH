#!/usr/bin/python

# KEITH Evolution base code for control, servo, motor and LED functions for the PiWars 2015 competition.
# Created by Harry Merckel, written for Python 2.7

# Import required Python libraries
import smbus, serial, time, numpy, random, sys, subprocess, math
from timeit import default_timer as timer
import RPi.GPIO as GPIO
from threading import Thread

# GPIO setup
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

# Global variables
global encoder1count, encoder2count
encoder1count = 0
encoder2count = 0

# The arduino class, including all the methods and functions for interfacing with the radio control receiver. (See arduino-receiver.ino).
class ArduinoReceiver(object):
    # Setting up the arduino connection for radio control
    def __init__(self):
        self.arduino = serial.Serial("/dev/ttyACM0",115200)
        self.correctionRatio        = 1.1
        self.upperDeadLimit         = 1440
        self.lowerDeadLimit         =  560
        self.middleDeadLower        =  960
        self.middleDeadUpper        = 1040
        self.radioChannelVertical   = 2
        self.radioChannelHorizontal = 1

    # Request for radio control channel statuses
    def rawRadio():
        controller = self.arduino.readline().split(" ")
        return controller

    # Deadzone checking for raw radio signals. Could be done for the final output, but I wanted to leave options open for all 5 channels.
    def isInDeadzone(self, input):
        if input > self.upperDeadLimit:                           return 1500
        if input < self.lowerDeadLimit:                           return  500
        if self.middleDeadUpper < input < self.middleDeadLower:   return 1000
        else:                                                     return input
        
    # Taking the two-axis controller input and turning it into differential drive values for the motors
    def controllerRead(self):
        motor1list = [] # Lists so the median, a more reliable average, can be used
        motor2list = []
        for i in range(3):
            controller = self.rawRadio()
            # Try-except for when the arduino or radio controller / receiver aren't working as expected - simpler than proper error handling...
            try:
                controllerx = ((float(controller[0])-1000)*1.1)+1000 # These scale up the range of the controller so the deadzones have more of an effect
                controllery = ((float(controller[2])-1000)*1.1)+1000
            except:
                controllerx = 1000
                controllery = 1000
            
            controllerx = self.isInDeadzone(controllerx) # Deadzone checking, ensures numbers aren't out of an acceptable range and helps make sure full
            controllery = self.isInDeadzone(controllery) #  forwards is full forwards and stop is stop.

            controllerx = (controllerx-1000)/5 # Scaling from a 500 to 1500 range to a -100 to +100 range
            controllery = (controllery-1000)/5

            controllerx = (controllerx/10)*abs(controllerx/10) # Changing the scaling on the steering axis to make it more sensitive at lower levels
            
            motor2 = controllery - controllerx # Converting the two controller axes into differential drive values
            motor1 = controllery + controllerx
            
            ratio1 = 100.0/abs(motor1) if (motor1 > 100 or motor1 < -100) else 1 # Scaling the results of the conversion back to acceptable levels
            ratio2 = 100.0/abs(motor2) if (motor2 > 100 or motor2 < -100) else 1 # This only needs to be done with axes that go all the way to the corners - 
            totalRatio = ratio1 * ratio2 #                                          if they only went in a circle this wouldn't be a problem.
            motor1 = totalRatio * motor1
            motor2 = totalRatio * motor2
            
            motor1list.append(motor1)
            motor2list.append(motor2)
            
        motor1list = sorted(m1list)
        motor2list = sorted(m2list)
        #print(int(m1otorlist[1]), int(motor2list[1]), rawRadio)
        return int(motor1list[1]), int(motor2list[1]), rawRadio # Taking the median value and returning it to the main loop


class robot(object): # The robot class, featuring all the Pi-only based code
    def __init__(self):
        # Setting up the GPIO pins
        self.motor1forwards  = 26
        self.motor1backwards = 19
        self.motor2forwards  = 20
        self.motor2backwards = 21
        self.trigger         = 5
        self.echo            = 6
        self.encoder1        = 17
        self.encoder2        = 27
        self.lineLeft        = 10
        self.lineMiddle      = 9
        self.lineRight       = 11
        self.servoPin        = 12

        GPIO.setup(self.motor1forwards,  GPIO.OUT) # Motor 1 - forwards
        GPIO.setup(self.motor1backwards, GPIO.OUT) # Motor 1 - backwards
        GPIO.setup(self.motor2forwards,  GPIO.OUT) # Motor 2 - forwards
        GPIO.setup(self.motor2backwards, GPIO.OUT) # Motor 2 - backwards

        #GPIO.setup(self.LED1,    GPIO.OUT) # LED for testing purposes
        #GPIO.setup(self.buzzer,  GPIO.OUT) # Buzzer for testing purposes
        GPIO.setup(self.trigger, GPIO.OUT) # HC-SR04 (ultrasonic module) trigger pin
        GPIO.setup(self.echo,    GPIO.IN)  # HC-SR04 echo pin
        #GPIO.setup(self.buttonPush, GPIO.IN) # LED button input
        #GPIO.setup(self.buttonLED,  GPIO.OUT) # LED button LED
        #self.buttonLEDcontrol = GPIO.PWM(self.buttonLED, 1) # LED in the control button
        #self.buttonLEDcontrol.start(0)
        GPIO.setup(self.lineLeft,GPIO.IN) # Line sensor inputs
        GPIO.setup(self.lineRight,GPIO.IN)
        GPIO.setup(self.lineMiddle,GPIO.IN)
        GPIO.setup(self.encoder1,GPIO.IN,pull_up_down=GPIO.PUD_UP) # Wheel encoder inputs
        GPIO.setup(self.encoder2,GPIO.IN,pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.servoPin, GPIO.OUT)

        # Setting up and starting PWM for motor speed control
        self.m1f = GPIO.PWM(self.motor1forwards,  500)
        self.m1b = GPIO.PWM(self.motor1backwards, 500)
        self.m2f = GPIO.PWM(self.motor2forwards,  500)
        self.m2b = GPIO.PWM(self.motor2backwards, 500)

        self.m1f.start(0)
        self.m1b.start(0)
        self.m2f.start(0)
        self.m2b.start(0)
        
        self.encodersEnabled = False
        
        """# MCP23017 I2C setup
        self.bus = smbus.SMBus(1)
        self.bus.write_byte_data(0x20,0x00,0b00000000)
        self.bus.write_byte_data(0x20,0x01,0b00000000)
        
        self.mcpA = 0x14
        self.mcpB = 0x15
        
        global MCPstatus
        MCPstatus = 0b00000000 # The initial MCP23017 output, with each bit being an output."""
        
        
            
        # This method is to simplify the motor control instead of having to use the ChangeDutyCycle function throughout the program.
    def motor(self, left, right):
        if left  >  100: left  = 100
        if left  < -100: left  = -100
        if right >  100: right = 100
        if right < -100: right = -100
        if left > 0:
            self.m1f.ChangeDutyCycle(left)
            self.m1b.ChangeDutyCycle(0)
        else:
            self.m1f.ChangeDutyCycle(0)
            self.m1b.ChangeDutyCycle(-left)
        if right > 0:
            self.m2f.ChangeDutyCycle(right)
            self.m2b.ChangeDutyCycle(0)
        else:
            self.m2f.ChangeDutyCycle(0)
            self.m2b.ChangeDutyCycle(-right)
            
    def mcp(self, side, value, overwrite=False): # A function for LED control
        global MCPstatus
        if overwrite:
            MCPstatus = value
        else:
            MCPstatus = MCPstatus ^ value # Bitwise XOR on the current MCP23017 status, so the previous value is retained unless the input has a 1, then the matching bit will be flipped.
            # For example, with a previous value of 0b00001101 and an input of 0b00000011 you get an output of 0b00001110 - the first two outputs have been swapped from on to off.
        try:
            self.bus.write_byte_data(0x20, 0x14 if side.lower() == 'a' else 0x15, value)
        except IOError: # Sometimes things don't quite work between the Pi and the MCP23017 - this resets the I2C bus, fixing that...
            subprocess.call(['i2cdetect', '-y', '1'])
    
    def ultrasonic(self, repetitions): # The code that runs the ultrasonic sensor. Very similar to last year's code because it simply works!
        distlist = [] # Making a blank list for the results

        # Taking a set of readings and taking the median for a slightly more reliable result.
        for i in range(repetitions):
            # Output a short pulse to start the measurement.
            GPIO.output(self.trigger, 1)
            time.sleep(0.00001)
            GPIO.output(self.trigger, 0)
            time0 = time.time()

            # Waiting for response to start
            time1=time0
            while ((GPIO.input(self.echo) == 0) and ((time1 - time0) < 0.02)):
                time1 = time.time()

            time1 = time.time()
            time2 = time1

            # Timing and waiting for return to finish
            while ((GPIO.input(self.echo) == 1) and ((time2 - time1) < 0.02)):
                time2 = time.time()
            time2 = time.time()

            time3 = (time2-time1)

            # Calculation of distance -> return time * speed of sound (m/s) / 2 (pulse has to go to and from object) * 100 (convert m to cm)
            currentDistance = time3*343/2*100
            distlist.append(currentDistance)
            time.sleep(0.01)
        distlist = sorted(distlist) # Sorting the distance readings and taking the median result, including handling for odd and even numbers of results.
        median = (distlist[((repetitions+1)//2)-1]) if float(repetitions/2.0) == float(repetitions//2) else ((distlist[(repetitions//2)-1]+distlist[(repetitions//2)])/2.0)
        return median # ^ takes the median result, or averages the two median results if there are an even number of iterations.
    
    def leftdetect(self): # Wheel encoder switch detection - adds one each time it detects a change from white to black or vice versa.
        global encoder1count
        lastStatus = 0
        while self.encodersEnabled:
            GPIO.setup(self.encoder1, GPIO.OUT) #Set your chosen pin to an output
            GPIO.output(self.encoder1, 1) #Drive the output high, charging the capacitor
            time.sleep(0.00001) #Charge capacitor
            time0 = timer()
            GPIO.setup(self.encoder1, GPIO.IN) # set pin to input
            time1=time0
            while GPIO.input(self.encoder1) == 1:
                time1 = timer()
            totalTime = time1-time0
            if lastStatus == 0 and totalTime > 0.002:
                encoder1count += 1
                lastStatus = 1
            if lastStatus == 0 and totalTime < 0.002:
                encoder1count += 1
                lastStatus = 1
            else:
                continue
        #print "Encoder 1: {0}".format(encoder1count)
        
    def rightdetect(self):
        global encoder2count
        lastStatus = 0
        while self.encodersEnabled:
            GPIO.setup(self.encoder2, GPIO.OUT) #Set your chosen pin to an output
            GPIO.output(self.encoder2, 1) #Drive the output high, charging the capacitor
            time.sleep(0.00001) #Charge capacitor
            time0 = timer()
            GPIO.setup(self.encoder2, GPIO.IN) # set pin to input
            time1=time0
            while GPIO.input(self.encoder2) == 1:
                time1 = timer()
            totalTime = time1-time0
            if lastStatus == 0 and totalTime > 0.002:
                encoder2count += 1
                lastStatus = 1
            if lastStatus == 0 and totalTime < 0.002:
                encoder2count += 1
                lastStatus = 1
            else:
                continue
        #print "Encoder 2: {0}".format(encoder1count)
        
    def startEncoders(self):
        global encoder1count, encoder2count
        encoder1count = 0
        encoder2count = 0
        #while True: # For testing purposes, to see if the encoders are actually working!
        #    print GPIO.input(self.encoder1)
        #    print GPIO.input(self.encoder2)
        if not self.encodersEnabled: # This is so multiple callbacks aren't added so this can be used to just reset the counts.
            #GPIO.add_event_detect(self.encoder1, GPIO.BOTH, callback=self.leftdetect) # Starts off the encoder reading with a GPIO edge detection event.
            #GPIO.add_event_detect(self.encoder2, GPIO.BOTH, callback=self.rightdetect)
            self.encodersEnabled = True
    
    def straightLine(self,distance,forwards=True): # This is proportional motor balancing. May add integral and derivative before event!
        self.motor(0, 0)
        global encoder1count, encoder2count
        self.startEncoders()
        while currentDistance < distance:
            # Encoder sensors are 18mm from the centre of the wheel, and there are 32 edges per rotation.
            # 2*pi*radius = circumference, divide that into 32 segments and multiply by how many ticks have passed.
            currentDistance = encoder1count * ((2 * math.pi * 1.8) / 32)
            difference = encoder2count - encoder1count
            if forwards:
                if difference == 0:
                    self.motor(100,100)
                if difference > 0:
                    self.motor(100-difference,100)
                if difference < 0:
                    self.motor(100,100+difference)
            else:
                if difference == 0:
                    self.motor(-100,-100)
                if difference > 0:
                    self.motor(-100,-100+difference)
                if difference < 0:
                    self.motor(-100-difference,-100)
        motor(0,0)
        time.sleep(0.2) # Delay to ensure KEITH has stopped before starting another movement
    
    def startServo(self):
        GPIO.setup(self.servoPin, GPIO.OUT)

        launchServo = GPIO.PWM(servo, 200)
        # Servo positions are based on pulse length: 1.00ms pulse is the neutral position, 0.50ms is low, 1.50ms is high
        # 200Hz PWM cycle is variable - it could be anywhere between 40Hz and 200Hz
        # 1s = 1000ms, so 100Hz makes each refresh 10ms wide. 5% = 0.5ms, 15% = 1.5ms, 10% = 1.0ms, so (with the default GPIO library) that's only 10 degrees of control.
        # 200Hz: 5ms width, 10% = 0.5ms, 30% = 1.5ms, 20% = 1.0ms - 20 degrees of control.
        PWMposition = 0
        servoPWM.start(PWMposition)
    
    def controlServo(self, position):
        # Set servo position, from 0 to 100
        PWMposition = (position//5)+10
        self.launchServo.ChangeDutyCycle(PWMposition)
                
    def distanceLoop(self, target1=5, target2=10, target3=20): # The code for the Proximity Alert challenge.
        self.motor(0, 0)
        distance = self.ultrasonic(5)
        print distance
        while distance >= target3: # Full speed ahead until less than target3 cm (20cm by default) from the wall.
            distance = self.ultrasonic(5)
            #print distance
            #self.straightLine()
            self.motor(100,100)
        while distance >= target2: # Proportionally slower speed between target3 and target2 distances.
            distance = self.ultrasonic(5)
            #print distance
            ratio = 100/target2
            self.motor(distance*ratio,distance*ratio)
        while distance >= target1: # Dead slow on final approach.
            distance = self.ultrasonic(5)
            #print distance
            self.motor(20,20)
        if distance < target1: # Stop!
            self.motor(0,0)
        #print distance
        self.motor(0,0)
    
    def manualControl(self, arduino):
        # Manual control using the RC controller. Includes servo control for skittles ball launcher.
        self.motor(0, 0)
        self.startServo(launch, self.launchPin, 50)
        while True:
            left, right, controller = arduino.controllerRead()
            try:
                switch = int(controller[4])
            except:
                switch = 500
            self.controlServo(launch, 100 if switch > 1000 else 0)
            motor(left, right)
    
    def lineFollow(self): # Code for the line following challenge! Very similar to last year's again because that also worked.
        self.motor(0, 0)
        while True:
            if self.ultrasonic(5) > 10:
                l = GPIO.input(self.lineLeft)
                c = GPIO.input(self.lineMiddle)
                r = GPIO.input(self.lineRight)
                # We are using 3 IR sensors and the program varies motor speed depending on what each sensor sees.
                if not l and c and not r:
                    self.motor(100,100)
                    continue
                if l and c and not r:
                    self.motor(-100,100)
                    continue
                if l and not c and not r:
                    self.motor(-70,100)
                    continue
                if not l and c and r:
                    self.motor(100,-100)
                    continue
                if not l and not c and r:
                    self.motor(100,-70)
                    continue
                if not l and not c and not r:
                    self.motor(100,100)
                    continue
                if l and c and r:
                    self.motor(0,0)
                    continue
            else:
                self.motor(0,0) # Stops if ultrasonic detects an obstacle (to protect the robot, not feet!)
                time.sleep(0.2)
              
    def threePointTurn(self): # The three point turn code - goes forwards, turns left, goes forwards, goes backwards, goes forwards, turns left and returns to the start. Simple?...
        straightLine(170)
        motor(-100,100)
        time.sleep(0.45)
        motor(0,0)
        time.sleep(0.2)
        straightLine(45)
        straightLine(90,False)
        starightLine(45)
        motor(-100,100)
        time.sleep(0.45)
        motor(0,0)
        time.sleep(0.2)
        straightLine(170)
        
    
    def button(self, receiver): # The code for method selection - press the button for less than a second and it'll change the selection, hold for more than a second and it'll start.
        finished = False
        selection = 1
        numberOfPrograms = 4
        while not finished:
            time1 = time.time()
            buttonState = GPIO.input(self.buttonPush)
            while buttonState == 1:
                totalTime = time.time() - time1
                if totalTime >= 1:
                    self.buttonLEDcontrol.ChangeDutyCycle(100)
                    toFinish = True # This isn't 'finished' as I want the program to only start when the button is released.
            if not toFinish:
                if selection == numberOfPrograms:
                    selection = 1
                else:
                    selection += 1
                self.buttonLEDcontrol.ChangeDutyCycle(50) # The button has an LED, so I decided to make it flash the current selection times per second.
                self.buttonLEDcontrol.ChangeFrequency(selection)
            else:
                self.buttonLEDcontrol.ChangeDutyCycle(0)
                finished = True
        if selection == 1: self.manualControl(receiver)
        if selection == 2: self.distanceLoop()
        if selection == 3: self.lineFollow()
        if selection == 4: self.threePointTurn()


if __name__ == "__main__":
    KEITH = robot()
    Receiver = ArduinoReceiver()
    
    KEITH.button(Receiver)
    
    print "Finishing"
    GPIO.cleanup()
