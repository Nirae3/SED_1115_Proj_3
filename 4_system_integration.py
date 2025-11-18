#############################################################################
#                     # 4 System Integration and Calibration
"""

System Integration & Coordination

Goal: Combine all modules into one complete interactive system.

Tasks:
Write the main loop that links everything:
read knobs → compute angles → move servos → lift/drop pen.
Handle initialization and error handling.
Add visual debugging (e.g., print positions to REPL).
Optionally add home/reset behavior or record simple motion patterns.
Output: The final working Etch-a-Sketch-style robot arm program.

"""
###############################################################################

def initialize_system() -> None: #type: ignore
    """
    initialise the system: servos, ADC pins
    """
    print("\n[INITIALIZING SYSTEM...]")

    # Example pin assignments (user-defined)
    shoulder_pin = 15
    elbow_pin = 16
    pen_pin = 14

    # Setup workspace boundaries via calibration
    workspace = map_paper()  # returns {x_min, x_max, y_min, y_max}

    system_state = {
        "shoulder_pin": shoulder_pin,
        "elbow_pin": elbow_pin,
        "pen_pin": pen_pin,
        "workspace": workspace,
        "arm1_len": 10.0,
        "arm2_len": 10.0,
        "last_angles": (0.0, 0.0),
        "drawing_enabled": False
    }

    print("[SYSTEM READY]")
    return system_state

def calibrate_servos() -> None: #type: ignore
    """
    move the phisical arm around the paper and calculate for the ranges where it can go
    perform basic callibrations
    for that, call 1_servo_control to perform the callibrations within 10 degrees 
    """
    print("\n[CALIBRATING SERVOS...]")

    list_of_angles = [0, 30, 60, 90, 120, 150, 180]
    calibration_shoulder = []
    calibration_elbow = []

    # ---- SHOULDER CALIBRATION ----
    for desired_angle in list_of_angles:
        duty_value = translate(desired_angle)
        shoulder_servo.duty_u16(duty_value)

        print(f"\nMoving shoulder servo to {desired_angle}°")
        time.sleep(1)

        try:
            actual_angle = float(input(f"Measured shoulder angle at {desired_angle}°: "))
        except:
            print("Invalid input. Skipping.")
            continue

        diff = actual_angle - desired_angle
        calibration_shoulder.append((desired_angle, actual_angle, diff))
        print(f"Recorded: Δ = {diff:+.2f}°")

    # ---- ELBOW CALIBRATION ----
    for desired_angle in list_of_angles:
        duty_value = translate(desired_angle)
        elbow_servo.duty_u16(duty_value)

        print(f"\nMoving elbow servo to {desired_angle}°")
        time.sleep(1)

        try:
            actual_angle = float(input(f"Measured elbow angle at {desired_angle}°: "))
        except:
            print("Invalid input. Skipping.")
            continue

        diff = actual_angle - desired_angle
        calibration_elbow.append((desired_angle, actual_angle, diff))
        print(f"Recorded: Δ = {diff:+.2f}°")

    print("\n[CALIBRATION COMPLETE]\n")

def update_arm_position() -> dict[str, float] #type: ignore
    """
    read inputs and update servo positions
    shoulder, elbow = get_target_angles()
    move_servo("shoudler", shoulder)
    """
     # Read potentiometers
    x_raw, y_raw = read_pot_values()

    # Normalize 0–65535 → 0.0–1.0
    x_norm, y_norm = normalize_pot_values(x_raw, y_raw)

    # Remove noise
    x_filtered, y_filtered = apply_deadzone(x_norm, y_norm)

    # Map normalized → physical coordinates
    x_pos, y_pos = map_to_coordinates(
        x_filtered, y_filtered,
        (system_state["workspace"]["x_min"], system_state["workspace"]["x_max"]),
        (system_state["workspace"]["y_min"], system_state["workspace"]["y_max"])
    )

    # Compute angles using IK
    shoulder_angle, elbow_angle = compute_joint_angles(
        x_pos, y_pos,
        system_state["arm1_len"],
        system_state["arm2_len"]
    )

    # Safety check
    if not safety_check(shoulder_angle, elbow_angle):
        print("[WARNING] Unsafe movement blocked.")
        return system_state["last_angles"]

    # Move servos
    move_shoulder(system_state["shoulder_pin"], shoulder_angle)
    move_elbow(system_state["elbow_pin"], elbow_angle)

    # Save new state
    system_state["last_angles"] = (shoulder_angle, elbow_angle)

    return {
        "shoulder": shoulder_angle,
        "elbow": elbow_angle
    }

def main_loop() ->  None #type: ignore
    """
    main loop that ties everything together
    """
    system_state = initialize_system()

    print("\n[ENTERING MAIN LOOP — Press Ctrl+C to exit]")

    while True:
        try:
            # Get new shoulder + elbow angles
            angles = update_arm_position(system_state)

            # Debug printout
            print(f"Shoulder: {angles['shoulder']:.2f}°,  Elbow: {angles['elbow']:.2f}°")

            time.sleep(0.01)  # loop speed

        except KeyboardInterrupt:
            print("\n[MAIN LOOP TERMINATED]")
            break
