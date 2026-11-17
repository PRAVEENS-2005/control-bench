# Multi-Domain Control Algorithm Benchmarking Toolkit

A Python-based state-space physics simulator designed to evaluate and benchmark advanced control algorithms across complex, non-linear robotic architectures. 

Built from first principles, this toolkit utilizes a **ROS 2-style YAML configuration file**, allowing engineers to seamlessly transition between testing 6-DOF drones, unstable balancing robots, and bipedal walkers without modifying the core physics engine.

##  Core Architecture
* **State-Space Physics Engine:** Dynamically generates multi-variable differential equations based on mechanical constraints (e.g., gravity, inertia, underactuation).
* **Cascaded Control Support:** Handles underactuated physics loops, such as translating X/Y position errors into target Roll/Pitch angles for 6-DOF drone flight.
* **YAML-Driven Testing:** Define target setpoints, simulation duration, variable counts, and sudden external disturbance vectors directly in `config.yaml`.
* **Dynamic Grid Plotting:** Automatically scales visualizations from a 1-axis graph to a 3x2 6-DOF grid depending on the robot loaded in the configuration.

---

##  Supported Control Algorithms

You can test up to 8 algorithms simultaneously by requesting them in the YAML config.

1. **PID:** Standard Proportional-Integral-Derivative feedback.
2. **ADRC:** Active Disturbance Rejection Control utilizing Extended State Observers (ESO).
3. **LQR:** Linear Quadratic Regulator (Optimal control for MIMO).
4. **MPC:** Model Predictive Control.
5. **LQG:** Linear Quadratic Gaussian.
6. **Feed-Forward Control:** Proactive load cancellation.
7. **Fuzzy Logic Control:** Linguistic rule-mapping.
8. **Adaptive Control:** Real-time gain adjustment.

---

## Supported Robot Physics Scenarios

1. **`drone_6_dof` (Full Quadcopter Kinematics)**
   * Simulates an underactuated system where X and Y positional movements require cascaded attitude control (Pitch/Roll).
2. **`drone_4_dof` (Attitude & Altitude)**
   * Direct coupling of Altitude, Roll, Pitch, and Yaw.
3. **`self_balancing_bot` (Inverted Pendulum)**
   * Simulates an inherently unstable system fighting gravity while attempting to traverse the X-axis.
4. **`bipedal_walker_lipm` (Linear Inverted Pendulum Model)**
   * Bipedal Center of Mass (CoM) tracking, managing hip height and step trajectory.

---

##  How to Run the Toolkit

### 1. Install Dependencies
```bash
pip3 install pyyaml numpy matplotlib
