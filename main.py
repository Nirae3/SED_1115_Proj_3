
#----------------------------------------------------------------------------------------

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

#----------------------------------------------------------------------------------------

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


----- Skills/Concepts -----


---- Concepts ----
Read X and Y potentiometers via ADC.
Filter/normalize readings (handle noise or drift).
Compute target shoulder and elbow angles using trigonometry (inverse kinematics).
Handle mapping (e.g., when the user twists the knobs, where does the pen move?).
Send these angles to the servo module.

"""
###############################################################################

#----------------------------------------------------------------------------------------

#############################################################################
#                # 3 Coordinate Calculation and Motion Planning
"""

----- Objective -----
Convert potentiometer input into Cartesian coordinates and then translate those coordinates
into joing angles.

----- Details -----
Use inverse kinematics to calculate the positions of the "shoulder" and "elbow" servos in 
relation to the X and Y positions. 
The system must ensure that the arm can reach any point within its workspacee based on
the X/Y inputs.
Handle edge cases where the input goes out of bounds or the arm is unable to reach
a specific point

----- Skills/Concepts -----
Inverse Kinematics calculation
Path planning (ensuring the arm moves smoothly to the target position)
Collision detection or boundary checks for movement limits

"""
###############################################################################

#----------------------------------------------------------------------------------------

#############################################################################
#                     # 4 System Integration and Calibration
"""

----- Objective -----
Integrate all components (servos, potentiometers, calculations) and callibrate
the system for accurate and smooth movement

----- Details -----
Test and tune the servos to ensure they behave as expected with the potentiometer inputs
Callibrate the system to map the potentiometer positions to the desired Cartesian coordinates
and joint angles
Fine-tune any motion issues, such as jittery movement, slow response, or overshooting of the
servo positions.
Add safety or error-handling mechanisms, such as ensuring the arm doesn't move out of bounds or over-extend.

----- Skills/Concepts -----
Debugging and 

"""
###############################################################################