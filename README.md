# Autonomous Drone Navigation using Deep Reinforcement Learning (DRL)

## Project Goal
The objective of this project is to develop and implement a Deep Reinforcement Learning (DRL) policy for a quadrotor UAV. The policy will enable high-precision hovering, navigation, and dynamic obstacle avoidance within a simulated environment.

## Technology and Platform
- **Simulator:** Webots (3D physics simulation and sensor integration)
- **Controller:** Python script interfacing with Webots (currently classical control)
- **Target Algorithm:** Proximal Policy Optimization (PPO)

## Current Achievement: Classical Control Baseline
The first phase focused on building a performance baseline using a traditional controller.

### 1. Classical Controller Implemented
A Proportional-only (P-Only) controller was created in Webots using IMU and GPS data to control altitude and attitude.

### 2. Baseline Test Results
The test attempted to maintain a stable hover.

| **Metric** | **P-Only Controller Result** | **Conclusion** |
|------------|-------------------------------|----------------|
| Hover Stability | Severe Overshoot (>100%) | Extreme instability and oscillatory behavior |
| Settling Time | Failure to Settle | The system did not stabilize during the test |

**Conclusion:** The P-only controller cannot manage the non-linear dynamics and high-dimensional control requirements of a quadrotor. This validates the need for a more adaptive control method based on Deep Reinforcement Learning (DRL).

## Next Steps
The project now transitions fully into the DRL phase:
- Define the reward function.
- Integrate Webots with a DRL pipeline using Python and Stable-Baselines3.
- Train the DRL agent to replace the classical controller.

## Setup (Controller)
To run the current P-Only controller in Webots:

- **Dependencies:** Webots and Python installed
- **Controller:** `controllers/manual_controller.py`
- **Run:** Open the world file in Webots and launch the simulation with the specified controller

