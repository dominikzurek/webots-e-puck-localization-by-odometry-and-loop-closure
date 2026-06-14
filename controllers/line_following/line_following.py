"""
    line_following controller.
    Author: Dominik Żurek - Poland

    This controller was developed as part of my coursework for the "Introduction to Robotics with Webots" 
    - Robot Localization by Odometry and Loop Closure Overview - Specialization on Coursera 
    from University of Colorado Boulder.
    LINK: https://www.coursera.org/specializations/introduction-robotics-webots

    What this code does:
    The robot follows a line on the arena floor using 3 ground sensors. 
    Simultaneously, it computes its 2D pose estimate (xw, yw, omega_z) using differential drive odometry.
    The simulation automatically terminates with high precision once the robot completes the track 
    and crosses the start/finish line region (only in simulation :) ).
"""

#  --- Imports ---
from controller import Robot
import numpy as np

#  --- Constants ---
MAX_SPEED = 6.28

#  --- Robot Initialize ---
robot = Robot()
timestep = int(robot.getBasicTimeStep())

left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

gs = []
for i in range(3):
    gs.append(robot.getDevice('gs'+str(i)))
    gs[-1].enable(timestep)

#  --- Variables and Initial Pose ---
xw = 0.0
yw = 0.0
omega_z = np.pi / 2

delta_t = timestep / 1000
localization_error = 0.0
stop = False

#  --- Main loop --- 
while robot.step(timestep) != -1:

    # ------------------ Motion Control ------------------
    # Read data from sensors
    g=[]
    for gsensor in gs:
        g.append(gsensor.getValue())

    left_sensor, middle_sensor, right_sensor = g[0], g[1], g[2]

    # Motion conditions
    if (xw > -0.090 and xw < -0.046 and yw > 0.090): # stop e-puck after line
        phildot, phirdot = MAX_SPEED, MAX_SPEED # block speed (drive straight)
        stop = True
    elif (left_sensor > 500 and middle_sensor < 350 and right_sensor > 500): # drive straight
        if stop:
            phildot, phirdot = 0.0, 0.0
        else: 
            phildot, phirdot = MAX_SPEED, MAX_SPEED
    elif (right_sensor < 550): # turn right
        phildot, phirdot = 0.20 * MAX_SPEED , -0.08 * MAX_SPEED
    elif (left_sensor < 550): # turn left
        phildot, phirdot = -0.08 * MAX_SPEED, 0.20 * MAX_SPEED

    # Write speed to wheels
    left_motor.setVelocity(phildot)
    right_motor.setVelocity(phirdot)

    # ------------------ Odometry Compute ------------------
    # Robot's linear displacement
    delta_x = ((0.0201 * (phildot + phirdot)) / 2) * delta_t

    # Transform displacement onto world axis
    xw = xw + np.cos(omega_z) * delta_x
    yw = yw + np.sin(omega_z) * delta_x

    # Update orientation around world Z axis
    delta_omega = ((0.0201 * (phirdot - phildot)) / 0.052) * delta_t
    omega_z += delta_omega

    # Compute distance from the origin
    localization_error = np.sqrt(xw**2 + yw**2)

    # ------------------ Console ------------------
    print(f"Position: X={xw:.3f}m, Y={yw:.3f}m | Localization error: {localization_error:.3f}m ~{localization_error*100:.1f}cm")

    # ------------------ Finish program ------------------
    if stop and phildot == 0.0 and phirdot == 0.0:
        break
    
    pass