
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

"""
###############################################################################

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