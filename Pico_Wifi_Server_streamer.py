import network
import socket
import time
from machine import Pin, PWM, ADC
import math
import ujson
import os
import random

# ---------- CONFIG ----------
PORT = 8765
L1 = 155.0
L2 = 155.0
DEBOUNCE_MS = 200
DEBUG = True
FILENAME = "ssid.txt"

LETTERS = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ"
DIGITS = "0123456789"
ALPHANUM = LETTERS + DIGITS

# ---------- HARDWARE SETUP ----------
# ADC for potentiometers (your physical arm control)
pot_x = ADC(Pin(26))  # X-axis potentiometer
pot_y = ADC(Pin(27))  # Y-axis potentiometer

# ADC for virtual arm (professor's code - for streaming)
adc_x = ADC(1)  # For shoulder streaming
adc_y = ADC(0)  # For elbow streaming
pen_button = Pin(22, Pin.IN, Pin.PULL_DOWN)

# PWM for servos (your physical arm)
shoulder = PWM(Pin(0), freq=50)
elbow = PWM(Pin(1), freq=50)
wrist = PWM(Pin(2), freq=50)

# ---------- YOUR ARM CONSTANTS ----------
ANGLE_OFFSET = -3  # calibration offset

# arm lengths (measured in cm)
len_shol_elbow = 17.0  # La
len_elbow_wrist = 14.0  # Lb

# Current servo angles
current_shoulder_angle = 0
current_elbow_angle = 0

# paper limits
X_MIN, X_MAX = 0, 29.0
Y_MIN, Y_MAX = 0.0, 21.0

# ---------- GLOBALS ----------
client_addr = None
pen_state = 0
last_button_state = 0
last_debounce = 0
last_announce = 0

# ---------- UTILITY ----------
def log(msg):
    if DEBUG:
        print(f"[PICO][{time.localtime()[3]:02d}:{time.localtime()[4]:02d}:{time.localtime()[5]:02d}] {msg}")

# ---------- SSID / PASSWORD ----------
def generate_SSID():
    """Prompt user for 3-letter prefix and generate SSID + random password"""
    while True:
        prefix = input("Enter 3-letter SSID prefix (A-Z or a-z only): ").strip()
        if len(prefix) == 3 and all(ch.isalpha() for ch in prefix):
            break
        print("Invalid input. Must be exactly 3 letters A-Z or a-z.")

    suffix = "{:05d}".format(random.randint(0, 99999))
    ssid = prefix + suffix
    password = "".join(random.choice(ALPHANUM) for _ in range(8))

    try:
        with open(FILENAME, "w") as f:
            f.write(f"{ssid}\n{password}\n")
        print(f"Created SSID: {ssid}, PASSWORD: {password}")
    except Exception as e:
        print(f"Error writing {FILENAME}: {e}")

    return ssid, password

def load_ssid_file():
    """Load SSID/password from file, or generate new if missing"""
    if FILENAME in os.listdir():
        try:
            with open(FILENAME, "r") as f:
                lines = [line.strip() for line in f if line.strip()]
                ssid = lines[0]
                password = lines[1] if len(lines) > 1 else generate_SSID()[1]
                log(f"Loaded SSID: {ssid}")
                return ssid, password
        except Exception as e:
            print(f"Error reading {FILENAME}: {e}")
            return generate_SSID()
    else:
        return generate_SSID()

# ---------- WIFI SETUP ----------
def setup_wifi(ssid, password):
    wlan = network.WLAN(network.AP_IF)
    wlan.active(True)
    wlan.config(essid=ssid, password=password)
    wlan.ifconfig(('192.168.4.1', '255.255.255.0', '192.168.4.1', '8.8.8.8'))
    log(f"WiFi AP active: SSID={ssid}, IP=192.168.4.1")
    return wlan

# ---------- NETWORK ----------
def setup_network():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("0.0.0.0", PORT))
    s.settimeout(0.05)
    log(f"UDP server listening on port {PORT}")
    return s

def handle_incoming_message(sock):
    """Register client and process KEEPALIVE"""
    global client_addr
    try:
        data, addr = sock.recvfrom(1024)
        msg = data.decode().strip()
        if msg == "HELLO":
            client_addr = addr
            sock.sendto(b"WELCOME", addr)
            log(f"Client {addr} registered")
        elif msg == "KEEPALIVE":
            client_addr = addr
    except OSError:
        pass

def stream_current_arm_state(sock, shoulder_angle, elbow_angle):
    """Send joint positions to client"""
    global client_addr, last_announce, pen_state
    if client_addr is None:
        return

    payload = ujson.dumps({
        "shoulder": shoulder_angle,
        "elbow": elbow_angle,
        "pen": pen_state
    })
    try:
        sock.sendto(payload.encode(), client_addr)
    except OSError as e:
        log(f"Send failed: {e}")
        client_addr = None
        return

    now = time.ticks_ms()
    if time.ticks_diff(now, last_announce) > 1000:
        log(f"Streaming to {client_addr}: S={shoulder_angle:.1f}, E={elbow_angle:.1f}, pen={pen_state}")
        last_announce = now

# ---------- YOUR ARM FUNCTIONS ----------
def read_pot(adc):
    """Getting rid of noise by reading the pot values of the average of the samples"""
    sample = 1
    total = 0
    for i in range(sample):
        total += adc.read_u16()
        time.sleep_ms(1)
    return total // sample

def map_pot_to_coordinate(pot_value, min_value, max_value):
    """Map potentiometer 0-65535 to coordinate range"""
    return min_value + (pot_value / 65535.0) * (max_value - min_value)

def translate(angle: float) -> int:
    """
    Converts an angle in degrees to the corresponding input
    for the duty_u16 method of the servo class.
    This prevents sending unsafe PWM values to the servo.
    """
    # apply the offset to the input angle
    adjusted_angle = angle + ANGLE_OFFSET

    # makes sure that the servo motors stays within bounds
    if adjusted_angle < 0:
        adjusted_angle = 0
    if adjusted_angle > 180:
        adjusted_angle = 180

    # minimum and maximum PWM in microseconds that correspond to the servo's physical limits
    pulse_min = 500  # 0 degrees
    pulse_max = 2500  # 180 degrees

    # calculate the PWM for the given angle
    pulse_width = pulse_min + (pulse_max - pulse_min) * (adjusted_angle / 180)

    # the period of a 50 Hz PWM signal is 20,000 microseconds
    duty_cycle = pulse_width / 20000

    # convert the duty cycle (0.0–1.0) into a 16-bit value for duty_u16
    duty_cycle_value = int(duty_cycle * 65535)

    return duty_cycle_value

def inverse_kinematics(x, y):
    """
    A = shoulder (at origin or offset position)
    B = elbow
    C = target position
    """
    
    Ax, Ay = 0.0, 0.0  # shoulder position, may need to change
    Cx, Cy = x, y  # target position
    
    La = len_shol_elbow
    Lb = len_elbow_wrist

    max_reach = La + Lb
    min_reach = abs(La - Lb)

    theta_S_offset = 0  # math.radians(45)  # adjust Shoulder servo mounting and convert to radians
    theta_E_offset = 0  # math.radians(20)  # adjust elbow servo mounting and convert to radians
    
    angle_AC = math.atan2(Cy, Cx)  # angle from horizontal x axis
    AC = math.sqrt(Cy**2 + Cx**2)  # distance from shoulder to target
    
    if AC > max_reach or AC < min_reach:  # check if can reach the target
        print(f"Target unreachable: AC={AC:.2f}, valid range=[{min_reach:.2f}, {max_reach:.2f}]")
        return None
    
    cos_BAC = (La**2 + AC**2 - Lb**2) / (2 * La * AC)  # shoulder angle
    cos_BAC = max(-1.0, min(1.0, cos_BAC))  # keep in the range between [-1, 1]
    angle_BAC = math.acos(cos_BAC)  # convert to radians

    cos_ABC = (La**2 + Lb**2 - AC**2) / (2 * La * Lb)  # elbow angle
    cos_ABC = max(-1.0, min(1.0, cos_ABC))  # keep in the range between [-1, 1]
    angle_ABC = math.acos(cos_ABC)  # convert to radians

    theta_AB = angle_AC - angle_BAC  # shoulder angle from horizontal line to the shoulder
    
    theta_S = theta_S_offset + theta_AB  # final shoulder angle including the offset
    theta_S_deg = math.degrees(theta_S)  # convert to degrees
    theta_S_deg = max(0, min(180, theta_S_deg))  # put in the range between [0, 180]

    theta_E = angle_ABC - theta_E_offset  # final elbow angle including offset
    theta_E_deg = math.degrees(theta_E)  # convert to degrees
    theta_E_deg = max(0, min(180, theta_E_deg))  # keep the range in between [0, 180]
    
    return (theta_S_deg, theta_E_deg)

def move_to(target_shoulder, target_elbow):
    global current_shoulder_angle, current_elbow_angle

    if current_shoulder_angle < target_shoulder:
        shoulder_range = range(int(current_shoulder_angle), int(target_shoulder) + 1)
    else:
        shoulder_range = range(int(current_shoulder_angle), int(target_shoulder), -1)

    if current_elbow_angle < target_elbow:
        elbow_range = range(int(current_elbow_angle), int(target_elbow) + 1)
    else:
        elbow_range = range(int(current_elbow_angle), int(target_elbow), -1)

    # Make ranges equal length
    max_len = max(len(shoulder_range), len(elbow_range))
    shoulder_list = list(shoulder_range)
    elbow_list = list(elbow_range)
    
    # Pad shorter list
    if len(shoulder_list) < max_len:
        shoulder_list.extend([shoulder_list[-1]] * (max_len - len(shoulder_list)))
    if len(elbow_list) < max_len:
        elbow_list.extend([elbow_list[-1]] * (max_len - len(elbow_list)))

    for s, e in zip(shoulder_list, elbow_list):
        shoulder.duty_u16(translate(s))
        elbow.duty_u16(translate(e))
        time.sleep(0.01)

    current_shoulder_angle = target_shoulder
    current_elbow_angle = target_elbow

def wrist_up():
    """Lift pen off paper"""
    wrist.duty_u16(translate(0))
    time.sleep(0.5)

def wrist_down():
    """Lower pen to paper"""
    wrist.duty_u16(translate(30))
    time.sleep(0.5)

# ---------- MAIN ----------
SSID, PASSWORD = load_ssid_file()
wlan = setup_wifi(SSID, PASSWORD)
sock = setup_network()
last_announce = 0

print("Etch-A-Sketch Starting...")
print(f"Workspace: X({X_MIN}, {X_MAX}), Y({Y_MIN}, {Y_MAX})")
move_to(0, 0)
print("Moved to origin (0,0)")
time.sleep(2)

while True:
    # Read potentiometer values for physical arm control
    pot_x_value = read_pot(pot_x)
    pot_y_value = read_pot(pot_y)
    
    # Map to coordinate space
    target_x = map_pot_to_coordinate(pot_x_value, X_MIN, X_MAX)
    target_y = map_pot_to_coordinate(pot_y_value, Y_MIN, Y_MAX)
    
    # Calculate inverse kinematics
    angles = inverse_kinematics(target_x, target_y)

    if angles:
        shoulder_ang, elbow_ang = angles
        print(f"Target: ({target_x:.1f}, {target_y:.1f}) ::: Shoulder={shoulder_ang:.1f} deg, Elbow={elbow_ang:.1f} deg")
        move_to(shoulder_ang, elbow_ang)
    else: 
        print(f"Target ({target_x:.1f}, {target_y:.1f}) is unreachable")

    # Read ADC values for virtual arm streaming (professor's code)
    shoulder_stream = 180.0 - (adc_x.read_u16() / 65535.0 * 180.0) - 90.0
    elbow_stream = 180.0 - (adc_y.read_u16() / 65535.0 * 180.0)

    # Pen state is simply the button's state (1 if pressed, 0 if not)
    pen_state = pen_button.value()

    # Send the current arm state (shoulder and elbow angles, pen state) to the client
    stream_current_arm_state(sock, shoulder_stream, elbow_stream)

    # Handle incoming messages (HELLO, KEEPALIVE)
    handle_incoming_message(sock)
    
    # Sleep for 50ms before repeating
    time.sleep(0.05)