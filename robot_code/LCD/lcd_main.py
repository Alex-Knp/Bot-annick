from lcd_api import LcdApi
from i2c_lcd import I2cLcd
 
I2C_ADDR = 0x78
I2C_NUM_ROWS = 2
I2C_NUM_COLS = 20
 
lcd = I2cLcd(1, I2C_ADDR, I2C_NUM_ROWS, I2C_NUM_COLS)
 
lcd.putstr("Great! It Works!")
lcd.move_to(3,1)
lcd.putstr("freva.com")