#libraries that enable communication with the pico and also manipulate time
from machine import Pin, PWM
import time

#setting a PWM signal and assigning it a frequency
pwm = PWM(Pin(0), freq=50, duty_u16=8192)
pwm.duty_u16(32768)

ANGLE_OFFSET = -3 #other angles to test calibration: -10, +5, +12

#setting PWM value for the servos
shoulder = PWM(Pin(0), freq=50)
elbow = PWM(Pin(1), freq=50)
wrist = PWM(Pin(2), freq=50)

# You should not modify the signature (name, input, return type) of this function
def translate(angle: float) -> int:
    """
    Converts an angle in degrees to the corresponding input
    for the duty_u16 method of the servo class.
    This prevents sending unsafe PWM values to the servo.
    """

    #apply the offset to the input angle
    adjusted_angle = angle + ANGLE_OFFSET

    #makes sure that the servo motors stays within bounds
    if adjusted_angle < 0:
        adjusted_angle = 0
    if adjusted_angle > 180:
        adjusted_angle = 180

    #minimum and maximum PWM in microseconds that correspond to the servo's physical limits
    pulse_min = 500 #0 degrees
    pulse_max = 2500 #180 degrees

    #calculate the PWM for the given angle
    pulse_width = pulse_min + (pulse_max - pulse_min) * (adjusted_angle / 180)

    #the period of a 50 Hz PWM signal is 20,000 microseconds
    duty_cycle = pulse_width / 20000

    #convert the duty cycle (0.0–1.0) into a 16-bit value for duty_u16
    duty_cycle_value = int(duty_cycle * 65535)

    print("angle =", angle, "adjusted_angle =", adjusted_angle, 
          "duty_cycle_value =", duty_cycle_value)

    return duty_cycle_value

#setting functions to control the movement of the pen servo
def wrist_up():
    wrist.duty_u16(translate(0)) #the angle at which the servo has the pen up
    time.sleep(0.5)
def wrist_down():
    wrist.duty_u16(translate(30)) #the angle at which the servo has the pen down
    time.sleep(0.5)

def move_to(x, y):
    """
    Move the shoulder and elbow servos to approximate x,y position.
    """
    shoulder_angle = x
    elbow_angle = y

    #moving the servos
    shoulder.duty_u16(translate(shoulder_angle))
    elbow.duty_u16(translate(elbow_angle))
    time.sleep(0.05)

#Using the button to control the wrist
button_sw5 = Pin(15, Pin.IN, Pin.PULL_DOWN)
# Main loop
while True:
    if button_sw5.value() == 1:
        # Move to pick position
        wrist_up()
        time.sleep(1)
        # Move to place position
        wrist_up()
        time.sleep(1)
        # Return to home position
        wrist_up()
        time.sleep(1)
    time.sleep(0.1)
