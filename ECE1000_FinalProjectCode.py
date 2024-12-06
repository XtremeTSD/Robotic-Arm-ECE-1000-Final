#Source for joystick configuration: https://www.tomshardware.com/how-to/raspberry-pi-pico-joystick
#Sources for using the motors: https://microcontrollerslab.com/servo-motor-raspberry-pi-pico-micropython/
#https://docs.micropython.org/en/latest/pyboard/tutorial/servo.html

from machine import Pin, ADC, PWM
import utime

xAxis = ADC(Pin(27))
yAxis = ADC(Pin(26))

button = Pin(16, Pin.IN, Pin.PULL_UP)

servo1 = PWM(Pin(1))
servo1.freq(50)
servo2 = PWM(Pin(15))
servo2.freq(50)
servo3 = PWM(Pin(6))
servo3.freq(50)


while True:
    xValue = xAxis.read_u16()
    yValue = yAxis.read_u16()
    buttonValue = button.value()
    if xValue <= 600:
        for position in range (1000, 9000, 50):
            servo1.duty_u16(position)
    elif xValue >= 60000:
        for position in range (9000, 1000, -50):
            servo1.duty_u16(position)
    if yValue <= 600:
        for position in range (1000, 9000, 50):
            servo2.duty_u16(position)
    elif yValue >= 60000:
        for position in range (9000, 1000, -50):
            servo2.duty_u16(position)
    if buttonValue == 0:#Button needs tested
        for position in range (1000, 9000, 50):
            
'''
Some code I might potentially use

while True:
    xValue = xAxis.read_u16()
    yValue = yAxis.read_u16()
    buttonValue = button.value()
    servo1.angle(0)
    servo2.angle(0)
    servo3.angle(0)
    sa1 = servo1.angle()
    sa2 = servo2.angle()
    sa3 = servo3.angle()
    while xValue <= 600 and servo1.angle() != -90:
        servo1.angle(sa1 - 1)
        sa1 = servo1.angle()
    while xValue >= 60000 and servo1.angle() != 90:
        servo1.angle(sa1 + 1)
        sa1 = servo1.angle()
    while yValue <= 600 and servo2.angle() != 90:
        servo2.angle(sa2 + 1)
        sa2 = servo2.angle()
    while yValue >= 60000 abd servo2.angle() != -90:
        servo2.angle(sa2 - 1)
        sa2 = servo2.angle()
    if buttonValue == 0;
        for position in range(1000,9000,50):
        pwm1.duty_u16(position)
        sleep(0.01)
    
'''
            

'''
Code from https://www.tomshardware.com/how-to/raspberry-pi-pico-joystick
from machine import Pin, ADC
import utime

xAxis = ADC(Pin(27))
yAxis = ADC(Pin(26))

button = Pin(16,Pin.IN, Pin.PULL_UP)

while True:
    xValue = xAxis.read_u16()
    yValue = yAxis.read_u16()
    buttonValue = button.value()
    xStatus = "middle"
    yStatus = "middle"
    buttonStatus = "not pressed"
    if xValue <= 600:
        xStatus = "left"
    elif xValue >= 60000:
        xStatus = "right"
    if yValue <= 600:
        yStatus = "up"
    elif yValue >= 60000:
        yStatus = "down"
    if buttonValue == 0:
        buttonStatus = "pressed"
    print("X: " + xStatus + ", Y: " + yStatus + " -- button " + buttonStatus)
    utime.sleep(0.1)
'''

'''
Code (mostly, I added some minor changes) from https://microcontrollerslab.com/servo-motor-raspberry-pi-pico-micropython/
from time import sleep
from machine import Pin, PWM

pwm1 = PWM(Pin(1))
pwm1.freq(50)
pwm2 = PWM(Pin(15))
pwm2.freq(50)

while True:
    for position in range(1000,9000,50):
        pwm1.duty_u16(position)
        pwm2.duty_u16(position)
        sleep(0.01)
    for position in range(9000,1000,-50):
        pwm1.duty_u16(position)
        pwm2.duty_u16(position)
        sleep(0.01)

'''
