#!/usr/bin/python
#--------------------------------------
#    ___  ___  _ ____
#   / _ \/ _ \(_) __/__  __ __
#  / , _/ ___/ /\ \/ _ \/ // /
# /_/|_/_/  /_/___/ .__/\_, /
#                /_/   /___/
#
#  lcd_i2c.py
#  LCD test script using I2C backpack.
#  Supports 16x2 and 20x4 screens.
#
# Author : Matt Hawkins
# Date   : 20/09/2015
#
# http://www.raspberrypi-spy.co.uk/
#
# Copyright 2015 Matt Hawkins
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#
#--------------------------------------
import smbus
import time
import wiringpi
from wiringpi import GPIO
wiringpi.wiringPiSetup()

# Define some device parameters
I2C_ADDR  = 0x27 # I2C device address
LCD_WIDTH = 20   # Maximum characters per line

# Define some device constants
LCD_CHR = 1 # Mode - Sending data
LCD_CMD = 0 # Mode - Sending command

LCD_LINE_1 = 0x80 # LCD RAM address for the 1st line
LCD_LINE_2 = 0xC0 # LCD RAM address for the 2nd line
LCD_LINE_3 = 0x94 # LCD RAM address for the 3rd line
LCD_LINE_4 = 0xD4 # LCD RAM address for the 4th line

LCD_BACKLIGHT  = 0x08  # On
#LCD_BACKLIGHT = 0x00  # Off

ENABLE = 0b00000100 # Enable bit

# Timing constants
E_PULSE = 0.0005
E_DELAY = 0.0005

#Open I2C interface
bus = smbus.SMBus(2)  # Rev 1 Pi uses 0 (and Orange PI PC, for pins 3 and 5)

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

def displayname(city):
  name = city["name"]
  lcd_string(f"{name}",LCD_LINE_1)

def displaycity(city):
  displayname(city)
  #lcd_string("                    ",LCD_LINE_2)
  numuses = city["numuses"]
  lcd_string(f"Pocet pouziti: {numuses:0>{5}}",LCD_LINE_3)
  last = city["last"]
  lcd_string(f"Naposledy: {last}",LCD_LINE_4)

def displayload():
  lcd_string("####################",LCD_LINE_3)
  lcd_string("####################",LCD_LINE_4)

def displayconn():
  lcd_string("     Pripojeno      ",LCD_LINE_3)
  lcd_string("                    ",LCD_LINE_4)

def main():
  lcd_init()
  wiringpi.pinMode(18, 0) # connect
  wiringpi.pinMode(10, 0) # next
  wiringpi.pinMode(13, 0) # previous

  #atlantis = {"name":">>>   Atlantis   <<<", "numuses":1, "last":"24.4.2025"}
  #rim = {"name":">>>     Rim      <<<", "numuses":50, "last":"17.1.2010"}

  cities = [
    {"name":">>>   Atlantis   <<<", "numuses":1, "last":"24.4.2025"},
    {"name":">>>     Rim      <<<", "numuses":50, "last":"17.1.2010"},
    {"name":">>>    Praha     <<<", "numuses":5000, "last":"10.6.2024"}
  ]
  x = 0

  while True:
    if 0 == wiringpi.digitalRead(10):
      x = x - 1
      while 0 == wiringpi.digitalRead(10):
        x = x
    if 0 == wiringpi.digitalRead(13):
      x = x + 1
      while 0 == wiringpi.digitalRead(13):
        x = x 
    if x < 0:
      x = len(cities) - 1
    if x >= len(cities):
      x = 0

    if 0 == wiringpi.digitalRead(18):
      displayname(cities[x])
      while 0 == wiringpi.digitalRead(18):
        displayload()
      while wiringpi.digitalRead(18):
        displayconn()
      while 0 == wiringpi.digitalRead(18):
        displaycity(cities[x])


    else:
      displaycity(cities[x])

    



  #  lcd_string(">>>   Atlantis   <<<",LCD_LINE_1)
  #  lcd_string("Vzdalenost:  ?????km",LCD_LINE_2)
  #  lcd_string("Pocet pouziti: 00001",LCD_LINE_3)
  #  lcd_string("Naposledy: 24.4.2025",LCD_LINE_4)


if __name__ == '__main__':

  try:
    main()
  except KeyboardInterrupt:
    pass
  finally:
    lcd_byte(0x01, LCD_CMD)



# GPIO pins on opi zero 2w
# +------+-----+----------+--------+---+  ZERO2W  +---+--------+----------+-----+------+
# | GPIO | wPi |   Name   |  Mode  | V | Physical | V |  Mode  | Name     | wPi | GPIO |
# +------+-----+----------+--------+---+----++----+---+--------+----------+-----+------+
# |      |     |     3.3V |        |   |  1 || 2  |   |        | 5V       |     |      |
# |  264 |   0 |    SDA.1 |    OFF | 0 |  3 || 4  |   |        | 5V       |     |      |
# |  263 |   1 |    SCL.1 |    OFF | 0 |  5 || 6  |   |        | GND      |     |      |
# |  269 |   2 |     PWM3 |    OFF | 0 |  7 || 8  | 0 | ALT2   | TXD.0    | 3   | 224  |
# |      |     |      GND |        |   |  9 || 10 | 0 | ALT2   | RXD.0    | 4   | 225  |
# |  226 |   5 |    TXD.5 |    OFF | 0 | 11 || 12 | 0 | OFF    | PI01     | 6   | 257  |
# |  227 |   7 |    RXD.5 |    OFF | 0 | 13 || 14 |   |        | GND      |     |      |
# |  261 |   8 |    TXD.2 |    OFF | 0 | 15 || 16 | 0 | OFF    | PWM4     | 9   | 270  |
# |      |     |     3.3V |        |   | 17 || 18 | 0 | OFF    | PH04     | 10  | 228  |
# |  231 |  11 |   MOSI.1 |    OFF | 0 | 19 || 20 |   |        | GND      |     |      |
# |  232 |  12 |   MISO.1 |    OFF | 0 | 21 || 22 | 0 | OFF    | RXD.2    | 13  | 262  |
# |  230 |  14 |   SCLK.1 |    OFF | 0 | 23 || 24 | 0 | OFF    | CE.0     | 15  | 229  |
# |      |     |      GND |        |   | 25 || 26 | 0 | OFF    | CE.1     | 16  | 233  |
# |  266 |  17 |    SDA.2 |    OFF | 0 | 27 || 28 | 0 | OFF    | SCL.2    | 18  | 265  |
# |  256 |  19 |     PI00 |    OFF | 0 | 29 || 30 |   |        | GND      |     |      |
# |  271 |  20 |     PI15 |    OFF | 0 | 31 || 32 | 0 | OFF    | PWM1     | 21  | 267  |
# |  268 |  22 |     PI12 |    OFF | 0 | 33 || 34 |   |        | GND      |     |      |
# |  258 |  23 |     PI02 |    OFF | 0 | 35 || 36 | 0 | OFF    | PC12     | 24  | 76   |
# |  272 |  25 |     PI16 |    OFF | 0 | 37 || 38 | 0 | OFF    | PI04     | 26  | 260  |
# |      |     |      GND |        |   | 39 || 40 | 0 | OFF    | PI03     | 27  | 259  |
# +------+-----+----------+--------+---+----++----+---+--------+----------+-----+------+
# | GPIO | wPi |   Name   |  Mode  | V | Physical | V |  Mode  | Name     | wPi | GPIO |
# +------+-----+----------+--------+---+  ZERO2W  +---+--------+----------+-----+------+