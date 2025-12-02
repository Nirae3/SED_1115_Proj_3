#libraries that enable communication with the pico and also manipulate time
from machine import Pin, PWM, ADC, I2C
import time, math
from ads1x15 import ADS1015

i2c = I2C(1, sda = Pin(14), scl = Pin(15))


#setting a PWM signal and assigning it a frequency
pwm = PWM(Pin(0), freq=50, duty_u16=8192)
pwm.duty_u16(32768)
pot_x = ADC(Pin(26))
pot_y = ADC(Pin(27))
external_ADC = ADS1015(i2c, 0x48, 1 )

ANGLE_OFFSET = -5 # create calibration table lookup to figure out the offset

#setting PWM value for the servos
shoulder = PWM(Pin(0), freq=50)
elbow = PWM(Pin(1), freq=50)
wrist = PWM(Pin(2), freq=50) 

# arm lengths (measured in cm)
len_shol_elbow = 17.0  # La
len_elbow_wrist = 14.0  # Lb

# Current servo angles
current_shoulder_angle = 90
current_elbow_angle = 90

# paper limits
X_MIN, X_MAX = 3.0, 27.0
Y_MIN, Y_MAX = -15.5, 15.0


def read_pot(adc):   # getting rid of noise by reading the pot values of the average of the samples
    sample = 1
    total = 0
    for i in range(sample):
        total += adc.read_u16()
        time.sleep_ms(1)
    return total // sample


def map_pot_to_coordinate(pot_value, min_value, max_value): # map potentiometer 0-65535 to coordinate range
    return min_value + (pot_value / 65535.0) * (max_value - min_value)


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

    return duty_cycle_value


def inverse_kinematics(x, y):
    """
    A = shoulder (at origin or offset position)
    B = elbow
    C = target position
    """
    
    Ax, Ay = 0.0, 0.0 # shoulder position, may need to change
    Cx, Cy = x, y # target position
    
    La = len_shol_elbow
    Lb = len_elbow_wrist

    max_reach = La + Lb
    min_reach = abs(La - Lb)

    theta_S_offset = math.radians(150)  # adjust Shoulder servo mounding and convert to radians
    theta_E_offset = math.radians(30)  # adjust elbow servo mounting and convert to raidans
    
    angle_AC = math.atan2(Cy, Cx)  # angle from horizontal x axis
    AC = math.sqrt(Cy**2 + Cx**2) # distance from shoulder to target
    
    if AC > max_reach or AC < min_reach: # check if can reach the target
        print(f"Target unreachable: AC={AC:.2f}, valid range=[{min_reach:.2f}, {max_reach:.2f}]")
        return None
    
    cos_BAC = (La**2 + AC**2 - Lb**2) / (2 * La * AC)  # shoulder angle
    cos_BAC = max(-1.0, min(1.0, cos_BAC))  # keep in the range between [-1, 1]
    angle_BAC = math.acos(cos_BAC)  # convert to radians

    cos_ABC = (La**2 + Lb**2 - AC**2) / (2 * La * Lb)  # shoulder angle
    cos_ABC = max(-1.0, min(1.0, cos_ABC))  # keep in the range between [-1, 1]
    angle_ABC = math.acos(cos_ABC)  # convert to radians

    theta_AB = angle_AC - angle_BAC # shoulder angle from horizontal line to the shoulder
    
    theta_S = theta_S_offset + theta_AB # final shoulder angle including the offset
    theta_S_deg = math.degrees(theta_S) # convert to degrees
    theta_S_deg = max(0, min(180, theta_S_deg)) # put in the range between [0, 180]


    theta_E = angle_ABC - theta_E_offset # final elbow angle including offset
    theta_E_deg = math.degrees(theta_E) # convert to degrees
    theta_E_deg = max(0, min(180, theta_E_deg)) # keep the range in between [0, 180]
    
    return (theta_S_deg, theta_E_deg)

def move_to(target_x, target_y):
    angles = inverse_kinematics(target_x, target_y)

    if angles is None:
        print(f"Target ({target_x:.1f}, {target_y:.1f}) is unreachable")
        return False

    shoulder_ang, elbow_ang = angles

    shoulder.duty_u16(translate(shoulder_ang))
    elbow.duty_u16(translate(elbow_ang))

    print(f"SHOULDER ANGLE: {elbow_ang}, ELBOW ANGLE: {elbow_ang}")

    time.sleep(0.5)
    return True

def wrist_up():
    """Lift pen off paper"""
    wrist.duty_u16(translate(0))
    time.sleep(0.5)


def wrist_down():
    """Lower pen to paper"""
    wrist.duty_u16(translate(30))
    time.sleep(0.5)


def main():
    print("Etch-A-Sketch Starting...")
    print(f"Workspace: X({X_MIN}, {X_MAX}), Y({Y_MIN}, {Y_MAX})")
    
    time.sleep(1)

    while True:
        # Read potentiometer values
        pot_x_value = read_pot(pot_x)
        pot_y_value = read_pot(pot_y)

        # Map to coordinate space
        target_x = map_pot_to_coordinate(pot_x_value, X_MIN, X_MAX)
        target_y = map_pot_to_coordinate(pot_y_value, Y_MIN, Y_MAX)

        print(f"Target: ({target_x:.1f}, {target_y:.1f})")

        # Let move_to handle IK + motion
        success = move_to(target_x, target_y)

        if success:
            raw_value = external_ADC.read(4, 1)
            print(raw_value)


if __name__ == "__main__":
    main()