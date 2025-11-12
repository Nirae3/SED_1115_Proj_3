
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

