import smbus
import time
import board
import busio
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd

#This is setup for the LCD Display
bus = smbus.SMBus(1)
i2c = busio.I2C(board.SCL, board.SDA)
lcd_columns = 16
lcd_rows = 2
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)

#This is the I2C address of the arduino
address = 0x04

def LCDDisplayAngle(angle):
    lcd.clear()
    lcd.color = [100, 100, 100]
    #time.sleep(1)
    #This branching statment checks we are getting a real value from the camera. If not, we assume it doesn't see anything
    if angle != None:
        #Due to the unique nature of the CV2 library and the dynamic data typing of Python, the type casts are added to 
        #ensure compatibility for whatever is passed through from the camera
        lcd.message = "MRKR Found " + "\nAngle: " + string(round(float(angle),2))
    else:
        lcd.message = "NO MRKR!"
    lcd.color = [0, 0, 0]
    #time.sleep(1)
    return 1
