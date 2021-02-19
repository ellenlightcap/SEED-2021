import smbus
import board
import busio

# This is the address we setup in the Arduino Program
address = 0x04

# for RPI version 1, use “bus = smbus.SMBus(0)”
bus = smbus.SMBus(1)

# Initialise I2C bus.
i2c = busio.I2C(board.SCL, board.SDA)

def writeNumber(value):
    #bus.write_byte(address, value)
    if value == None:
        return -1
    try:
        bus.write_byte_data(address, 0, value)
    except:
        return -1
    return 0

def readNumber():
    #number = bus.read_byte(address)
    number = bus.read_byte_data(address, 0)
    return number
