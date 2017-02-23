
import smbus
import time
import serial
import sys 
import RPi.GPIO as GPIO 
from time import sleep 
import Adafruit_DHT 
import urllib2 
myAPI = "KCFJ4LY0NDGXWCXV"
# Define some device parameters
I2C_ADDR  = 0x27 # I2C device address
LCD_WIDTH = 16   # Maximum characters per line
ser = serial.Serial('/dev/ttyACM0',9600)
# Define some device constants
LCD_CHR = 1 # Mode - Sending data
LCD_CMD = 0 # Mode - Sending command

LINE_1 = 0x80 # LCD RAM address for the 1st line
LINE_2 = 0x89 # LCD RAM address for the 2nd line
LINE_3 = 0xC0 # LCD RAM address for the 3rd line
LINE_4 = 0xC9
LCD_BACKLIGHT  = 0x08  # On
#LCD_BACKLIGHT = 0x00  # Off
RH = 0
T = 0
W = 0
ENABLE = 0b00000100 # Enable bit
# Timing constants
E_PULSE = 0.0005
E_DELAY = 0.0005

#Open I2C interface
#bus = smbus.SMBus(0)  # Rev 1 Pi uses 0
bus = smbus.SMBus(1) # Rev 2 Pi uses 1
def getSensorData(): 
   RH, T = Adafruit_DHT.read_retry(Adafruit_DHT.DHT22, 23) 
   return (str(RH), str(T))
def dht22(): 
   print 'starting...' 
   baseURL = 'https://api.thingspeak.com/update?api_key=%s' % myAPI 
   while True: 
       try: 
           RH, T = getSensorData() 
           f = urllib2.urlopen(baseURL + 
                               "&field1=%s&field2=%s" % (T, RH)) 
           print f.read() 
           f.close() 
           sleep(300) #uploads DHT22 sensor values every 5 minutes 
       except: 
           print 'exiting.' 
           break
def lcd_init():
  # Initialise display
  lcd_byte(0x33,LCD_CMD) # 110011 Initialise
  lcd_byte(0x32,LCD_CMD) # 110010 Initialise
  lcd_byte(0x06,LCD_CMD) # 000110 Cursor move direction
  lcd_byte(0x0C,LCD_CMD) # 001100 Display On,Cursor Off, Blink Off 
  lcd_byte(0x28,LCD_CMD) # 101000 Data length, number of lines, font size
  lcd_byte(0x01,LCD_CMD) # 000001 Clear display
  time.sleep(E_DELAY)

def lcd_byte(bits, mode):
  # Send byte to data pins
  # bits = the data
  # mode = 1 for data
  #        0 for command

  bits_high = mode | (bits & 0xF0) | LCD_BACKLIGHT
  bits_low = mode | ((bits<<4) & 0xF0) | LCD_BACKLIGHT

  # High bits
  bus.write_byte(I2C_ADDR, bits_high)
  lcd_toggle_enable(bits_high)

  # Low bits
  bus.write_byte(I2C_ADDR, bits_low)
  lcd_toggle_enable(bits_low)

def lcd_toggle_enable(bits):
  # Toggle enable
  time.sleep(E_DELAY)
  bus.write_byte(I2C_ADDR, (bits | ENABLE))
  time.sleep(E_PULSE)
  bus.write_byte(I2C_ADDR,(bits & ~ENABLE))
  time.sleep(E_DELAY)

def lcd_string(message,line):
  # Send string to display

  message = message.ljust(LCD_WIDTH," ")

  lcd_byte(line, LCD_CMD)

  for i in range(LCD_WIDTH):
    lcd_byte(ord(message[i]),LCD_CHR)

def main():
  # Main program block
  # Initialise display
  lcd_init()
  read_serial = ser.readline()
  while True:
    dht22()
    W = ser.readline()    
    # Send some test
    print s
    lcd_string("TEMPERATURE:",LINE_1)
    lcd_string(str(T),LINE_3)
    time.sleep(3)
    lcd_string("HUMIDITY:",LINE_1)
    lcd_string(str(RH),LINE_3)
    time.sleep(3)
    lcd_string("WATER LEVEL:",LINE_1)
    lcd_string(str(W),LINE_3)
    time.sleep(3)

if __name__ == '__main__':

  try:
    main()
  except KeyboardInterrupt:
    pass
  finally:
    lcd_byte(0x01, LCD_CMD)

