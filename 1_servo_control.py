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
