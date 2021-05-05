import smbus
import time
import time
import board
import busio
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
# for RPI version 1, use “bus = smbus.SMBus(0)”
bus = smbus.SMBus(1)

# Modify this if you have a different sized Character LCD
lcd_columns = 16
lcd_rows = 2

# Initialise I2C bus.
i2c = busio.I2C(board.SCL, board.SDA)

# Initialise the LCD class
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)

# This is the address we setup in the Arduino Program
address = 0x04

def writeNumber(value,offset):
    #bus.write_byte(address, value)
    try:
        bus.write_byte_data(address, offset, value)
    except:
        return 0
    return -1

def readNumber():
    #number = bus.read_byte(address)
    number = bus.read_byte_data(address, 0)
    return number

while True:
    off = input("Enter offset (0/1): ")
    var = input("Enter 1 – 9: ")
    if not var:
        continue

    if(writeNumber(int(var), int(off)) == 0):
        print("I2C Error")
        lcd.clear()
        lcd.color = [100, 100, 100]
        time.sleep(1)
        # Print two line message
        lcd.message = "I2C Error"
    else:
        print("RPI: Hi Arduino, I sent you ", var)
        # sleep one second
        time.sleep(1)
    
        number = readNumber()
        print("Arduino: Here is the new number: ", number)

        lcd.clear()
        lcd.color = [100, 100, 100]
        time.sleep(1)
        # Print two line message
        lcd.message = "sent: " + str(var) + "\ngot: " + str(number)
    # Wait 2s
    time.sleep(2)
    lcd.color = [0, 0, 0]
    lcd.clear()
    
