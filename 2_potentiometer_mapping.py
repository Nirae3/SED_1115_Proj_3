
#############################################################################
#                     # 2 Input Processing (Potentiometer Mapping)
"""

----- Objective -----
Read and map potentiometer values to control the X and Y movements of the arm

----- Details -----
Use ADC (analog-to-digital converter) pins on the Pico to read potentiometer values.
Smooth and normalize readings (e.g., convert 0-65535 range to servo angles).
Implement logic that converts X and Y changes into meaningful shoulder/elbow angle changes.
Possibly include sensitivity control or dead zones for smoother motion.



---- Concepts ----
Read X and Y potentiometers via ADC.
Filter/normalize readings (handle noise or drift).
Compute target shoulder and elbow angles using trigonometry (inverse kinematics).
Handle mapping (e.g., when the user twists the knobs, where does the pen move?).
Send these angles to the servo module.


potentiometer pwm value -> (X, Y) coordinates -> (Angle, Angle) to reach the coordinates -> Arm moves
"""
###############################################################################

"""
potX = get x pwm values max x can go: 65565 min x can go, 300
potY = get Y pwm values. max y can go: 65565 min y can go, 300
"""

from machine import Pin, ADC, PWM
import time

servo_shoulder = PWM(Pin(0))
servo_elbow = PWM(Pin(1))

def calibrate():
    X_MIN = print(f"press to record lowest X value")
    X_MAX = print(f"press to record highest X value")
    Y_MIN = print(f"press to record lowest X value")
    Y_MAX = print(f"press to record highest Y value")
    BOARD_WIDTH = print(f"measure the board width")
    BOARD_HIGHT = print(f"measure the board height")
    
potX = ADC(Pin(26))
potY = ADC(Pin(27))


X_MIN = None
X_MAX = None
Y_MIN = None
Y_MAX = None

btn_min = Pin(14, Pin.IN, Pin.PULL_UP)
btn_max = Pin(15, Pin.IN, Pin.PULL_UP)

#############################################################
#                   CALLIBRATION                            #
#############################################################
def wait_for_press(button):
    while button.value() == 1:
        time.sleep(0.01)   # wait until pressed
    while button.value() == 0:
        time.sleep(0.01)   # wait for release

def calibrate_axis(axis_name, adc):
    print("Move", axis_name, "to MIN and press min button")
    wait_for_press(btn_min)
    MIN = adc.read_u16()
    print(axis_name, "MIN =", MIN)

    print("Move", axis_name, "to MAX and press max button")
    wait_for_press(btn_max)
    MAX = adc.read_u16()
    print(axis_name, "MAX =", MAX)

    return MIN, MAX

def calibrate():
    global X_MIN, X_MAX, Y_MIN, Y_MAX
    print("---- CALIBRATING X ----")
    X_MIN, X_MAX = calibrate_axis("X", potX)

    print("---- CALIBRATING Y ----")
    Y_MIN, Y_MAX = calibrate_axis("Y", potY)

    print("Calibration complete.")
    print("X:", X_MIN, X_MAX)
    print("Y:", Y_MIN, Y_MAX)


### MAP ADC TO A COORDINATE ###
def map_adc_to_coord(value, min_val, max_val, max_coord):
    # Clamp range
    if value < min_val: value = min_val
    if value > max_val: value = max_val

    # Linear scaling
    fraction = (value - min_val) / (max_val - min_val)
    return fraction * max_coord

### inverse kinematics
def inverse_kinematics():
    """
    put inverse kinematics logic here
    """

def read_pot_values() -> tuple[int, int]: #type: ignore
    """
    returns a tuple (x_raw, y_raw) in range (0, 65535)
    """
    pass

def normalize_pot_values(x_raw: int, y_raw:int) -> tuple[float, float] # type: ignore
    """
    return normalized values between 0.0 and 1.0
    """
    pass


def apply_deadzone(x_norm: float, y_norm: float, threshold: float = 0.05) -> tuple[float, float] #type: ignore
    """
    ignore small changed in the potentiometer,
    ignore noise up to +- 5 potentiometer values
    """
    pass

def map_to_coordinates(x_filtered: float, y_filtered: float, x_range: tuple[float, float], y_range: tuple[float, float]) -> tuple[float, float] # type: ignore
    """
    map normalized potentiometer values to physical (X,Y) positions
    x and y ranges represent the physical limmits of the arm's reach in x and y directions.
    ex: x_pos, y_pos = map_to_coordinates(0.6, 0.4, (-10, 10), (0, 15))
    """
    pass

def compute_joint_angles(x_pos: float, y_pos: float, arm1_len: float, arm2_len: float) -> tuple[float, float] # type: ignore
    """
    using inverse kinematics from servo_control part 1, compute the joint angles
    incorporate the length of the arm to calculate for the coordinates in degreees.
    """
    pass


def get_target_angles() -> tuple[float, float] #type: ignore
    """
    runs all the steps: read plot -> normalize -> map -> compute angles
    """
    pass


while True:
    potX_raw=adc.read()
    potY_raw=adc.read()
    print(f"Assign minimum/maximum buttons")
    min_button_assign = Pin(1, Pin.IN, Pin.PULL_UP)
    max_button_assign = Pin(2, Pin.IN, Pin.PULL_UP)