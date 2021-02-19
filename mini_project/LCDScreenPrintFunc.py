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

def LCDDisplay(desired,actual):
    lcd.clear()
    lcd.color = [100, 100, 100]
    #time.sleep(1)
    lcd.message = "Setpoint: " + desired + "\nPostion: " + actual
    lcd.color = [0, 0, 0]
    #time.sleep(1)
    return 1
    
