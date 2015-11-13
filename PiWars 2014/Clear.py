import smbus
import time

# Simply clears the LEDs.

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

mcpwritea(255)
mcpwriteb(255)
time.sleep(1)
mcpwritea(0)
mcpwriteb(0)