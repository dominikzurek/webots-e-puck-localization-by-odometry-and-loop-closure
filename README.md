# e-Puck Robot: Line Following & 2D Pose Estimation via Odometry

This repository contains a Python-based robot controller developed as part of my coursework for the **"Introduction to Robotics with Webots"** specialization (specifically the *Robot Localization by Odometry and Loop Closure Overview* module) offered by the **University of Colorado Boulder** via Coursera.

## Project Overview

The objective of this project is to implement a robust reactive line-following controller for the differential-drive **e-puck** robot in a Webots simulation environment. Simultaneously, the controller computes the robot's real-time 2D pose using forward kinematics and mathematical integration of odometry data. 

The simulation is designed to automatically terminate with high precision once the track is fully completed.

## Project Requirements

To successfully pass this laboratory assignment, the controller had to strictly satisfy the following engineering constraints:
1. **Line Following:** The robot must autonomously follow the black track using only 3 ground-facing infrared sensors (`gs0`, `gs1`, `gs2`).
2. **2D Kinematic Odometry:** The code must continuously calculate the robot's dead reckoning pose ($x_w, y_w, \omega_z$) using differential drive forward kinematics.
3. **Automated Termination:** The simulation must automatically bring the robot to a complete stop ($v_L = 0, v_R = 0$) and exit the loop immediately after crossing the start/finish line upon lap completion.

### Key Features
* **Reactive Navigation:** Line following utilizing 3 infrared ground sensors with dynamic adjustment for straight lines and sharp turns.
* **Dead Reckoning Odometry:** Real-time computation of linear displacement and angular orientation based on the differential drive kinematic model.
* **Deterministic Spatial Triggering:** Automated simulation cutoff using geographical coordinate boundaries optimized for high repeatability.
  
## Execution Steps

* **Step 0:** Read and process data from 3 infrared ground sensors (`gs0`, `gs1`, `gs2`).
* **Step 1:** Execute reactive line-following control with dynamic speed corrections for sharp turns.
* **Step 2:** Compute real-time linear displacement ($\Delta x$) and angular orientation updates ($\Delta\omega$).
* **Step 3:** Estimate global 2D world coordinates ($x_w, y_w$) and monitor Euclidean distance from the origin.
* **Step 4:** Trigger an automated, high-precision stop sequence once the start/finish line region is crossed (only in simulation :) ).
  
## Results & Performance

* **Target Constraint:** The course required the final localization error to remain under **20.0 cm**.
* **Achieved Accuracy:** The controller consistently achieves a final localization error of **~18.0 cm**, fully satisfying the top-tier grading criteria.

```text
[Console Output Example]
Position: X=-0.068m, Y=0.124m | Localization error: 0.180m ~18.0cm
Robot successfully crossed the finish line and came to a complete stop.
```
## Technologies Used

* **Webots:** Robot simulation environment.
* **Python / NumPy:** Controller programming, forward kinematics, and numerical integration.
* **Webots e-puck model:** Differential drive robot equipped with 3 IR ground sensors.
