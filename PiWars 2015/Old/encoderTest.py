import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BCM)
pin = 17

def Read():
    GPIO.setup(pin, GPIO.OUT) #Set your chosen pin to an output
    GPIO.output(pin, 1) #Drive the output high, charging the capacitor
    time.sleep(0.1) #wait for the cap to charge. Equations available on wikipedia for working out exact times
    time0 = time.time()
    GPIO.setup(pin, GPIO.IN) # set pin to input
    time1=time0
    while GPIO.input(pin) == 1:
        time1 = time.time()
    return time1-time0

while True:
    Read()
    print Read()
    time.sleep(0.25)