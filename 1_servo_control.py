###########################################################################
                        # 1 Servo Control and Kinematics
"""
----- Objective -----
Control the three servos

----- Details -----
Control the three servos based on input values from shoulder, elbow joints.
This will involve calculating the angles and ensuring smooth transitions between them.

----- Concepts -----
Create functions such as move_shoulder(pin, angle), set_joint_positions(shoulder, elbow)
Calibrate servo angles to match physical positions of the arm (angle-to-duty-cycle mapping)
Hangle servo speed and range limits to avoid mechanical strain
"""
###############################################################################

from machine import ADC, Pin, PWM
import time

#PWM setup
shoulder_servo = PWM(Pin(16))
shoulder_servo.freq(50)

#PWM setup
elbow_servo = PWM(Pin(17))
elbow_servo.freq(50)

#Potentiometer setup
shoulder_pot = ADC(Pin(26))
elbow_pot    = ADC(Pin(27))

def translate(angle: float) -> int:
    """
    Converts an angle (0° to 180°) into a 16-bit PWM duty cycle value.
    This tells the servo how far to rotate.
    """
    duty_min = 1638   #the minimum value for the duty_cycle
    duty_max = 8192   #the maximum value for the duty_cycle
    duty_value = int(duty_min + (duty_max - duty_min) * angle / 180)
    return duty_value

def read_pot_as_angle(pot: ADC) -> int:
    """
    Reads a potentiometer and converts it to 0–180 degrees
    """
    raw = pot.read_u16()  # 0–65535
    angle = int((raw / 65535) * 180)
    return angle

def move_shoulder(angle: float) -> None:
    """
    Controls the shoulder servo motor by converting an angle into a PWM signal.
    """
    shoulder_servo.duty_u16(translate(angle))
    
def move_elbow(angle: float) -> None:
    """
    Controls the elbow servo based on target angle, ensuring safe motion range.
    """
    elbow_servo.duty_u16(translate(angle))

def move_both_servos(shoulder_angle: int, elbow_angle: int) -> None:
    """
    Moves both shoulder and elbow servos simultaneously to reach target joint positions.
    """
    shoulder_servo.duty_u16(translate(shoulder_angle))
    elbow_servo.duty_u16(translate(elbow_angle))
    time.sleep(0.02)

#while the code is running, the servo should recieve PWM signals from the potentiometers
while True:
    shoulder_angle = read_pot_as_angle(shoulder_pot)
    elbow_angle = read_pot_as_angle(elbow_pot)

    move_both_servos(shoulder_angle, elbow_angle)

#overlaps with potentiometer mapping section. 
def calculate_inverse_kinematics(x, y, arm_length1, arm_length2) -> tuple[float, float] #type: ignore
    """
    Uses inverse kinematics to compute shoulder and elbow angles for a given (x, y) position.
    """
    pass
