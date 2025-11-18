from machine import Pin, PWM
import time
# Safe angles
PEN_UP_ANGLE = 90
PEN_DOWN_ANGLE = 45
MIN_ANGLE = 0
MAX_ANGLE = 180
# Convert angle to duty for servo
def angle_to_duty(angle):
    duty = int((angle / 180) * 9000 + 1000)
    return duty
# Safety check function
def safety_check(shoulder_angle: int, elbow_angle: int) -> bool:
	#Ensures joint angles remain within safe operational limits.
	if not (MIN_ANGLE <= shoulder_angle <= MAX_ANGLE):
		print("ERROR: Shoulder angle out of bounds!")
		return False
	if not (MIN_ANGLE <= elbow_angle <= MAX_ANGLE):
		print("ERROR: Elbow angle out of bounds!")
		return False
	return True
# Safe pen setting function
def safe_set_pen(angle: int, moving: bool = False) -> None:
    # Don't drop pen while moving
    if moving and angle == PEN_DOWN_ANGLE:
        print("ERROR: Cannot put pen DOWN while moving!")
        return
	# c_a is current angles, t_a is target angles
def plan_path(c_a, t_a, steps, pen_down = True) -> list[tuple[float, float]] #type: ignore
	path: list[tuple[float, float]] = []
	for i in range(steps + 1):
		t = i / steps
		interp_shoulder = c_a[0] + (t_a[0] - c_a[0]) * t
		interp_elbow = c_a[1] + (t_a[1] - c_a[1]) * t
		if pen_down == 1:
			safe_set_pen(PEN_DOWN_ANGLE)
		else:
			safe_set_pen(PEN_UP_ANGLE)
		path.append((interp_shoulder, interp_elbow))
	return path
# Map paper function
def map_paper() -> tuple[str, float]: # type: ignore
	axis_limits = "X: 0-200mm, Y: 0-150mm"
	area = float(200 * 150)  # in square millimeters
	return (axis_limits, area)
# Convert angle to duty cycle for servo
def translate(angle: float) -> int:
	# Converts an angle in degrees to the corresponding input
	# for the duty_u16 method of the servo class
	MIN = 1638 # 0 degrees
	MAX = 8192 # 180 degrees
	DEG = (MAX - MIN) / 180 # value per degree
	# clamp angle to be between 0 and 180
	angle = max(0, min(180, angle))
	return int(angle * DEG + MIN)
# Pen raising and lowering functions
def pen_lift() -> bool #type: ignore
	#Raises the pen by setting the servo to the up position.
	up_angle = 90  # Define the angle for pen up position
	duty_cycle = translate(up_angle)
	# Code to set the servo to duty_cycle for pen up
	print(f"Pen raised to angle {up_angle} with duty cycle {duty_cycle}")
def pen_down():
	#Lowers the pen by setting the servo to the down position.
	down_angle = 0  # Define the angle for pen down position
	duty_cycle = translate(down_angle)
	# Code to set the servo to duty_cycle for pen down
	print(f"Pen lowered to angle {down_angle} with duty cycle {duty_cycle}")
# Handling some errors cases	
try:
	pen_lift()
	pen_down()
except Exception as e:
	print(f"An error occurred: {e}")
# Define drawing_more function
def drawing_more() -> bool #type: ignore
	return True
