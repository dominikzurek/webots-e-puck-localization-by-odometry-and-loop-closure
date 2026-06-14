# e-Puck Robot: Line Following & 2D Pose Estimation via Odometry

This repository contains a Python-based robot controller developed as part of my coursework for the **"Introduction to Robotics with Webots"** specialization (specifically the *Robot Localization by Odometry and Loop Closure Overview* module) offered by the **University of Colorado Boulder** via Coursera.

## Project Overview

The objective of this project is to implement a robust reactive line-following controller for the differential-drive **e-puck** robot in a Webots simulation environment. Simultaneously, the controller computes the robot's real-time 2D pose ($x_w, y_w, \omega_z$) using forward kinematics and mathematical integration of odometry data. 

The simulation is designed to automatically terminate with high precision once the track is fully completed.

### Key Features
* **Reactive Navigation:** Line following utilizing 3 infrared ground sensors with dynamic adjustment for straight lines and sharp turns.
* **Dead Reckoning Odometry:** Real-time computation of linear displacement and angular orientation based on the differential drive kinematic model.
* **Deterministic Spatial Triggering:** Automated simulation cutoff using geographical coordinate boundaries ($x_w, y_w$) optimized for high repeatability.

---

## Kinematics & Mathematical Model

The odometry updates are computed at each physics timestep ($\Delta t = \text{timestep} / 1000$) using the following discrete-time kinematic equations:

1. **Linear Displacement ($\Delta x$):**
   $$\Delta x = \frac{R \cdot (\dot{\phi}_L + \dot{\phi}_R)}{2} \cdot \Delta t$$
   *Where $R = 0.0201\text{ m}$ (wheel radius) and $\dot{\phi}$ represents wheel angular velocities.*

2. **Global Position Update ($x_w, y_w$):**
   $$x_w = x_w + \cos(\omega_z) \cdot \Delta x$$
   $$y_w = y_w + \sin(\omega_z) \cdot \Delta x$$

3. **Angular Orientation Update ($\Delta\omega$):**
   $$\Delta\omega = \frac{R \cdot (\dot{\phi}_R - \dot{\phi}_L)}{L} \cdot \Delta t$$
   *Where $L = 0.052\text{ m}$ (wheelbase axle length).*

---

## Results & Performance

* **Target Constraint:** The course required the final localization error to remain under **20.0 cm**.
* **Achieved Accuracy:** The controller consistently achieves a final localization error of **~18.0 cm**, fully satisfying the top-tier grading criteria.

```text
[Console Output Example]
Position: X=-0.068m, Y=0.124m | Localization error: 0.180m ~18.0cm
Robot successfully crossed the finish line and came to a complete stop.
