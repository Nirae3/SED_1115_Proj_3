import network
import socket
import time
import machine
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

    # Read ADC values (0-65535) and scale to a meaningful range
    # Flipping the direction of the angles by subtracting from 180
    # Adding 90-degree offset for shoulder angle (clockwise)
    shoulder = 180.0 - (adc_x.read_u16() / 65535.0 * 180.0) - 90.0  # Flip and offset shoulder
    elbow = 180.0 - (adc_y.read_u16() / 65535.0 * 180.0)  # Flip elbow

    # Pen state is simply the button's state (1 if pressed, 0 if not)
    pen_state = pen_button.value()

    # Send the current arm state (shoulder and elbow angles, pen state) to the client
    stream_current_arm_state(sock, shoulder, elbow)

    # Sleep for 50ms before repeating
    time.sleep(0.05)
    
    # Handle incoming messages (HELLO, KEEPALIVE)
    handle_incoming_message(sock)
