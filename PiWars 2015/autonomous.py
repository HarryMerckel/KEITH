import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BCM)

m1fpin = 26
m1bpin = 19
m2fpin = 20
m2bpin = 21

GPIO.setup(m1fpin,GPIO.OUT)
GPIO.setup(m2fpin,GPIO.OUT)

m1f = GPIO.PWM(m1fpin,500)
m2f = GPIO.PWM(m2fpin,500)

m1f.start(0)
m2f.start(0)

m1f.ChangeDutyCycle(100)
m2f.ChangeDutyCycle(99.9) # Fudge factor

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
oneM = 211 # No. ticks for 1m of movement

while counter < int(oneM*12):
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
"""
while True:
    print Read()
    time.sleep(0.005)

m1f.ChangeDutyCycle(0)
m2f.ChangeDutyCycle(0)
time.sleep(0.5)
GPIO.cleanup()
time.sleep(0.5)
"""
