#############################################################################
#                    # 1 Servo Control and Kinematics
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

def move_shoulder(pin: PIN, angle: float) -> PWM: # type: ignore
    """
    Controls the shoulder servo motor by converting an angle into a PWM signal.
    
    """
    shoulder_servo = PWM(Pin(16))
    shoulder_servo.freq(50)
    return

def move_elbow(pin: PIN, angle: float) -> PWM: # type: ignore
    """
    Controls the elbow servo based on target angle, ensuring safe motion range.
    """
    pass



def set_joint_positions(shoulder_angle: int, elbow_angle: int) -> None #type: ignore
    """
    Moves both shoulder and elbow servos simultaneously to reach target joint positions.
    """
    pass

def calculate_inverse_kinematics(x, y, arm_length1, arm_length2) -> tuple[float, float] #type: ignore
    """
    Uses inverse kinematics to compute shoulder and elbow angles for a given (x, y) position.
    """
    pass
