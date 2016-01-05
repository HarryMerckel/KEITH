import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BCM)

m1fpin = 26
m1bpin = 19
m2fpin = 20
m2bpin = 21

GPIO.setup(m1fpin,GPIO.OUT)
GPIO.setup(m2fpin,GPIO.OUT)
GPIO.setup(m1bpin,GPIO.OUT)
GPIO.setup(m2bpin,GPIO.OUT)

m1f = GPIO.PWM(m1fpin,500)
m2f = GPIO.PWM(m2fpin,500)
m1b = GPIO.PWM(m1bpin,500)
m2b = GPIO.PWM(m2bpin,500)

m1f.start(0)
m2f.start(0)
m1b.start(0)
m2b.start(0)

#m1f.ChangeDutyCycle(100)
#m2f.ChangeDutyCycle(99.75)

#m1b.ChangeDutyCycle(100)
#m2f.ChangeDutyCycle(100)

pin = 17

def Read():
    GPIO.setup(pin,GPIO.OUT)
    GPIO.output(pin,1)
    time.sleep(0.00001)
    time0=time.time()
    GPIO.setup(pin,GPIO.IN)
    time1=time0
    while GPIO.input(pin)==1:
        time1 = time.time()
    return True if time1-time0 > 0.0003 else False if time1-time0 < 0.0001 else None
    #return time1-time0

lastStatus = False
counter = 0
oneM = 211
ninetyd = 37

def stopAll():
    m1f.ChangeDutyCycle(0)
    m2f.ChangeDutyCycle(0)
    m1b.ChangeDutyCycle(0)
    m2b.ChangeDutyCycle(0)

stopAll()

def goUntil(ticks,turn=False,back=False):
    counter = 0
    lastStatus = False
    if turn == True:
        m1b.ChangeDutyCycle(100)
        m2f.ChangeDutyCycle(100)
    else:
        if back == False:
            m1f.ChangeDutyCycle(100)
            m2f.ChangeDutyCycle(99.7)
        else:
            m1b.ChangeDutyCycle(99.7)
            m2b.ChangeDutyCycle(100)
    while counter < ticks:
        status = Read()
        if lastStatus == True and status == False:
            lastStatus = False
            counter += 1
        if lastStatus == False and status == True:
            lastStatus = True
            counter += 1
    stopAll()
    time.sleep(0.2)

goUntil(1.6*oneM,False,False)
goUntil(ninetyd, True, False)
goUntil(0.3*oneM,False,False)
goUntil(0.65*oneM,False,True )
goUntil(0.35*oneM,False,False)
goUntil(ninetyd, True, False)
goUntil(1.65*oneM,False,False)
""".
while counter < ninetyd:
    status = Read()
    if lastStatus == True and status == False:
        lastStatus = False
        counter += 1
        print counter
    if lastStatus == False and status == True:
        lastStatus = True
        counter += 1
        print counter
    #time.sleep(0.01)

while True:
    print Read()
    time.sleep(0.005)
"""
