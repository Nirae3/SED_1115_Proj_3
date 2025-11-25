import network
import socket
import time
from machine import Pin, PWM, ADC
import ujson
import os
import random
import time, math

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
# ADC and pen control related setup
adc_x = machine.ADC(1)  # For shoulder
adc_y = machine.ADC(0)  # For elbow
pen_button = machine.Pin(22, machine.Pin.IN, machine.Pin.PULL_DOWN)

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

def stream_current_arm_state(sock, shoulder, elbow):
    """Send joint positions to client"""
    global client_addr, last_announce, pen_state
    if client_addr is None:
        return

    payload = ujson.dumps({
        "shoulder": shoulder,
        "elbow": elbow,
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
        log(f"Streaming to {client_addr}: S={shoulder:.1f}, E={elbow:.1f}, pen={pen_state}")
        last_announce = now

# ---------- MAIN ----------
SSID, PASSWORD = load_ssid_file()
wlan = setup_wifi(SSID, PASSWORD)
sock = setup_network()
last_announce = 0
now = 0

while True:

    #setting a PWM signal and assigning it a frequency
    pwm = PWM(Pin(0), freq=50, duty_u16=8192)
    pwm.duty_u16(32768)
    pot_x = ADC(Pin(26))
    pot_y = ADC(Pin(27))


    ANGLE_OFFSET = -3 #other angles to test calibration: -10, +5, +12

    #setting PWM value for the servos
    shoulder = PWM(Pin(0), freq=50)
    elbow = PWM(Pin(1), freq=50)
    wrist = PWM(Pin(2), freq=50)

    # arm lengths
    len_shol_elbow = 20.0
    len_elbow_wrist = 20.0
    len_wrist_end = 5.0

    # paper limits
    X_MIN, X_MAX = -20.0, 20.0
    Y_MIN, Y_MAX = 0.0, 25.0


    """def calibration():
        print("=== ENTERING CALIBRATION MODE ===")
        print ("this will move the arm to test positions")
        print("measure the X and Y coordinates where the pen is")
        print("use a ruler from the choulder to the pen tip")
        test_positions = [
            (45, 90, 90, "LEFTMOST - Maximum left reach"),
            (135, 90, 90, "RIGHTMOST - Maximum right reach"),
            (90, 45, 90, "CLOSEST - Nearest to shoulder"),
            (90, 135, 90, "FARTHEST - Maximum forward reach"),
            (90, 90, 90, "CENTER - Middle position"),
        ]

        
        for i, (s,e) in enumerate(test_positions):
            print (f"\n Test {i+1}: Shoulder = {s}, Elbow = {e}")
            move_to(s,e)
            input("Move arm to this position. Measure X and Y. Press Enter...")
        
        measuremets=[]"""


    def read_pot(adc):   # getting rid of noize by reading the pot values of the average of the samples
        sample = 5
        total = 0
        for i in range(sample):
            total += adc.read_u16()
            time.sleep_ms(1)
        return total // sample

    def map_pot_to_coordinate (pot_value, min_value, max_value): # map potentiometer 0-65535 to coordinate range
        return min_value + (pot_value / 65535.0) * (max_value - min_value)
    # pot_value/65535 = converts to 0-1 range
    # max_val - min_val = total range of the paper
    # (pot_value/65535) *   max_val-min_val = find the pot value in the given range
    # min_value + (pot_value / 65535.0) * (max_value - min_value) = final coordinate accounting for min != 0


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

        # print("angle =", angle, "adjusted_angle =", adjusted_angle, "\n duty_cycle_value =", duty_cycle_value)

        return duty_cycle_value



    def inverse_kinematics (x, y):
        """
        A = shouder at origin 0,0 or at (Ax, Ay)
        C = target (x,y) => (Cx, Cy)
        B = elbow -> we calccualte this
        """

        Ax, Ay = 0.0, 0.0
        Cx, Cy = x, y

        AC = math.sqrt((Ax - Cx)**2 + (Ay - Cy)**2) # distance from shoulder to terget

        max_reach = len_shol_elbow + len_elbow_wrist
        min_reach = abs(len_shol_elbow - len_elbow_wrist)

        if AC > max_reach or AC < min_reach:
            print(f"Target is unrecable: AC = {AC}, valid range is[{min_reach}, {max_reach}]")
            return None
        
        Abase_c = math.sqrt((Ax-Cx)**2 + Cy**2) # base distance
        cos_BAC = (len_shol_elbow**2 + AC**2 - len_elbow_wrist**2)/ (2* len_shol_elbow * AC)
        cos_BAC = max(-1.0, min(1.0, cos_BAC)) # put im a valid range
        angle_BAC = math.acos(cos_BAC) # converting to radians

        sin_ACB = (len_shol_elbow * math.sin(angle_BAC)) / len_elbow_wrist
        sin_ACB = max(-1, min(1.0, sin_ACB)) # put in a valid range
        angle_ACB = math.asin(sin_ACB) # converting to radians

        if Ay == 0:
            angle_YAC = math.atan2(Cx, Cy)
        else: 
            cos_YAC = (Ay**2 + AC**2 - Abase_c**2) / (2*Ay * AC)
            cos_YAC = max(-1.0, min(1.0, cos_YAC))
            angle_YAC = math.acos(cos_YAC)

        alpha = angle_BAC + angle_YAC # shoulder angle in kinematic chain
        beta = angle_BAC + angle_ACB # elbow angle in kinematic chain

        alpha_deg = math.degrees(alpha) # convert to degrees
        beta_deg = math.degrees(beta) # convert to degrees
    
    # convert to servo angles with mounting offsets
    # may need to be adjusted using experementation

        servoA = alpha_deg - 75 
        servoB = 150 - beta_deg
        servoC = 90

        servoA = max(0, min(180, servoA)) # keep within t he range
        servoB = max(0, min(180, servoB))
        servoC = max(0, min(180, servoC))

        return(servoA, servoB, servoC)

    def move_to(shoulder_angle, elbow_angle, wrist_angle):
        shoulder.duty_u16(translate(shoulder_angle))
        elbow.duty_u16(translate(elbow_angle))
        wrist.duty_u16(translate(wrist_angle))


    #setting functions to control the movement of the pen servo
    def wrist_up():
        wrist.duty_u16(translate(0)) #the angle at which the servo has the pen up
        time.sleep(0.5)
    def wrist_down():
        wrist.duty_u16(translate(30)) #the angle at which the servo has the pen down
        time.sleep(0.5)



    def main():
        print("hi")

        move_to(90,90,90)

        while True:
            pot_x_value = read_pot(pot_x)
            pot_y_value = read_pot(pot_y)

            target_x = map_pot_to_coordinate (pot_x_value, X_MIN, X_MAX)
            target_y = map_pot_to_coordinate (pot_y_value, Y_MIN, Y_MAX)

            angles = inverse_kinematics(target_x, target_y)

            if angles:
                shoulder, elbow, wrist = angles
                print(f"Target: ({target_x}, {target_y}) - > Angles: Shoulder = {shoulder} deg, Elbow = {elbow} deg, Wrist: {wrist}")
                move_to(shoulder, elbow, wrist)
            else: 
                print(f"target ({target_x}, {target_y} is unreachable)")

            time.sleep(1)

    if __name__ == "__main__":
        main()


        # Pen state is simply the button's state (1 if pressed, 0 if not)
        pen_state = pen_button.value()

        # Send the current arm state (shoulder and elbow angles, pen state) to the client
        stream_current_arm_state(sock, shoulder, elbow)

        # Sleep for 50ms before repeating
        time.sleep(0.05)
        
        # Handle incoming messages (HELLO, KEEPALIVE)
        handle_incoming_message(sock)
