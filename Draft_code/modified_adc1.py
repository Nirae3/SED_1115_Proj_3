# I2C signal pins
I2C_SDA = 14
I2C_SCL = 15

# ADS 1015 configuration information
ADS1015_ADDR = 0x48
ADS1015_PWM = 2     # port 2 has (low-pass filtered) PWM signal
ADS1015_A_FB = 0
ADS1015_B_FB = 1

from machine import Pin, ADC, I2C
# You will need to upload the file ads1x15.py to the pico for the following code to work properly
from ads1x15 import ADS1015
from time import sleep
i2c = I2C(1, sda=Pin(I2C_SDA), scl=Pin(I2C_SCL))
adc = ADS1015(i2c, ADS1015_ADDR, 1)

# scan i2c bus to get a listing of all I2C devices... including the ADC that has the servo feedback values
addresses = i2c.scan()
for address in addresses:
	print(hex(address))


# Read filtered PWM input from the header
while True:
	valueA = adc.read(1, ADS1015_A_FB)
	valueB = adc.read(1, ADS1015_B_FB)
	print("A: ", valueA, ", B: ", valueB)
	sleep(0.1)


## a 1461   B: 1330
### a 874   704
### 394   621
### 4591282