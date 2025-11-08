# controllers/manual_controller_metrics.py

from controller import Robot
import numpy as np

# --- 1. CONFIGURATION ---
TIME_STEP = 32
SIMULATION_DURATION = 20.0  # Run for 20 seconds to gather metrics

# --- Physics & Targets ---
HOVER_VELOCITY = 56.0
TARGET_ALTITUDE = 1.0
start_position_xy = None

# --- PID GAINS (Proven Stable) ---
KP_ALTITUDE, KI_ALTITUDE, KD_ALTITUDE = 1.0, 0.02, 6.0
KP_ATTITUDE, KD_ATTITUDE = 10.0, 7.0
KP_YAW, KD_YAW = 10.0, 5.0

# --- Flight State & Limits ---
is_hovering = False
TAKEOFF_THRUST = HOVER_VELOCITY + 4.0
TAKEOFF_THRESHOLD = 0.90
altitude_integral = 0.0

# --- METRICS TRACKING ---
metrics = {
    "rise_time_start": None, "rise_time_10": None, "rise_time_90": None,
    "peak_altitude": 0.0, "settling_time": None, "settled": False,
    "settling_start_time": None, "steady_state_error_sum": 0.0, "steady_state_samples": 0
}
SETTLING_TOLERANCE_BAND = 0.05  # Settle when within +/- 5% of target
SETTLING_DURATION = 2.0  # Must stay in band for 2 seconds

# --- 2. INITIALIZATION ---
robot = Robot()
timestep = int(robot.getBasicTimeStep())
start_time = robot.getTime()

# --- 3. DEVICE ACQUISITION ---
imu, gps, gyro = robot.getDevice("imu"), robot.getDevice("gps"), robot.getDevice("gyro")
imu.enable(TIME_STEP);
gps.enable(TIME_STEP);
gyro.enable(TIME_STEP)
motors = {'front_left': robot.getDevice('m4_motor'), 'front_right': robot.getDevice('m1_motor'),
          'rear_left': robot.getDevice('m3_motor'), 'rear_right': robot.getDevice('m2_motor')}
for motor in motors.values(): motor.setPosition(float('inf')); motor.setVelocity(0.0)

robot.step(timestep)
print("[Info] METRICS GATHERING: Running stable PID controller for 20 seconds.")

# --- 4. MAIN CONTROL LOOP ---
while robot.step(timestep) != -1:
    current_time = robot.getTime()
    if current_time - start_time > SIMULATION_DURATION: break

    dt = timestep / 1000.0
    roll, pitch, _ = imu.getRollPitchYaw()
    x_pos, y_pos, altitude = gps.getValues()
    vertical_velocity = gps.getSpeedVector()[2]
    roll_rate, pitch_rate, yaw_rate = gyro.getValues()

    if start_position_xy is None: start_position_xy = [x_pos, y_pos]

    # --- Flight Logic (Identical to previous stable version) ---
    if not is_hovering and altitude >= TARGET_ALTITUDE * TAKEOFF_THRESHOLD: is_hovering = True; altitude_integral = 0.0
    if is_hovering:
        altitude_error = TARGET_ALTITUDE - altitude;
        altitude_integral = np.clip(altitude_integral + altitude_error * dt, -0.5, 0.5)
        vertical_thrust_adjustment = (KP_ALTITUDE * altitude_error) + (KI_ALTITUDE * altitude_integral) - (
                    KD_ALTITUDE * vertical_velocity)
        base_thrust = HOVER_VELOCITY + vertical_thrust_adjustment
    else:
        base_thrust = TAKEOFF_THRUST
    roll_adj = KP_ATTITUDE * (0.0 - roll) - KD_ATTITUDE * roll_rate;
    pitch_adj = KP_ATTITUDE * (0.0 - pitch) - KD_ATTITUDE * pitch_rate;
    yaw_adj = KP_YAW * (0.0 - yaw_rate)
    m_fl = base_thrust - pitch_adj + roll_adj + yaw_adj;
    m_fr = base_thrust - pitch_adj - roll_adj - yaw_adj;
    m_rl = base_thrust + pitch_adj + roll_adj - yaw_adj;
    m_rr = base_thrust + pitch_adj - roll_adj + yaw_adj
    velocities = [m_fl, m_fr, m_rl, m_rr];
    motors['front_left'].setVelocity(np.clip(velocities[0], 0, 4000));
    motors['front_right'].setVelocity(np.clip(velocities[1], 0, 4000));
    motors['rear_left'].setVelocity(np.clip(velocities[2], 0, 4000));
    motors['rear_right'].setVelocity(np.clip(velocities[3], 0, 4000))

    # --- METRICS CALCULATION LOGIC ---
    # Rise Time
    if metrics["rise_time_start"] is None and altitude > 0.01: metrics["rise_time_start"] = current_time
    if metrics["rise_time_10"] is None and altitude >= TARGET_ALTITUDE * 0.1: metrics["rise_time_10"] = current_time
    if metrics["rise_time_90"] is None and altitude >= TARGET_ALTITUDE * 0.9: metrics["rise_time_90"] = current_time
    # Peak Altitude
    if is_hovering: metrics["peak_altitude"] = max(metrics["peak_altitude"], altitude)
    # Settling Time & Steady-State Error
    if is_hovering and not metrics["settled"]:
        if abs(altitude - TARGET_ALTITUDE) <= TARGET_ALTITUDE * SETTLING_TOLERANCE_BAND:
            if metrics["settling_start_time"] is None:
                metrics["settling_start_time"] = current_time
            elif current_time - metrics["settling_start_time"] >= SETTLING_DURATION:
                metrics["settling_time"] = current_time - metrics["rise_time_start"];
                metrics["settled"] = True
        else:
            metrics["settling_start_time"] = None
    if metrics["settled"]:
        metrics["steady_state_error_sum"] += abs(altitude - TARGET_ALTITUDE);
        metrics["steady_state_samples"] += 1

# --- 5. FINAL REPORT ---
for motor in motors.values(): motor.setVelocity(0.0)  # Stop drone
print("\n--- PID CONTROLLER PERFORMANCE REPORT ---")
# Rise Time
if metrics["rise_time_10"] and metrics["rise_time_90"]:
    rise_time = metrics["rise_time_90"] - metrics["rise_time_10"]
    print(f"Rise Time (10% to 90%): {rise_time:.2f} seconds")
else:
    print("Rise Time: Could not be determined.")
# Overshoot
overshoot = ((metrics["peak_altitude"] - TARGET_ALTITUDE) / TARGET_ALTITUDE) * 100
print(f"Peak Altitude: {metrics['peak_altitude']:.2f} m")
print(f"Overshoot: {max(0, overshoot):.2f}%")
# Settling Time
if metrics["settling_time"]:
    print(f"Settling Time: {metrics['settling_time']:.2f} seconds")
else:
    print(f"Settling Time: Did not settle within {SIMULATION_DURATION}s.")
# Steady-State Error
if metrics["steady_state_samples"] > 0:
    sse = (metrics["steady_state_error_sum"] / metrics["steady_state_samples"]) * 100
    print(f"Steady-State Error: {sse:.2f}%")
else:
    print("Steady-State Error: Not calculated (did not settle).")
# Total Drift
final_pos = gps.getValues()
total_drift = np.sqrt((final_pos[0] - start_position_xy[0]) ** 2 + (final_pos[1] - start_position_xy[1]) ** 2)
print(f"Total Horizontal Drift in {SIMULATION_DURATION}s: {total_drift:.2f} m")
print("-----------------------------------------\n")