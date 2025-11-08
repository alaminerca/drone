# controllers/manual_controller.py

from controller import Robot
import numpy as np

# --- 1. CONFIGURATION ---
TIME_STEP = 32

# --- Physics & Targets ---
HOVER_VELOCITY = 56.0
TARGET_ALTITUDE = 1.0

print(f"[Info] Using hover velocity: {HOVER_VELOCITY:.2f} rad/s")

# --- PID GAINS (Simplified & Re-Tuned for Altitude Stability) ---
KP_ALTITUDE = 1.0
KI_ALTITUDE = 0.02
KD_ALTITUDE = 6.0

KP_ATTITUDE = 10.0
KD_ATTITUDE = 7.0
KP_YAW = 10.0
KD_YAW = 5.0

# --- Flight State & Limits ---
is_hovering = False
TAKEOFF_THRUST = HOVER_VELOCITY + 4.0
TAKEOFF_THRESHOLD = 0.90
altitude_integral = 0.0

# --- 2. INITIALIZATION ---
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# --- 3. DEVICE ACQUISITION ---
imu, gps, gyro = robot.getDevice("imu"), robot.getDevice("gps"), robot.getDevice("gyro")
imu.enable(TIME_STEP);
gps.enable(TIME_STEP);
gyro.enable(TIME_STEP)

motors = {
    'front_left': robot.getDevice('m4_motor'),  # CW
    'front_right': robot.getDevice('m1_motor'),  # CCW
    'rear_left': robot.getDevice('m3_motor'),  # CCW
    'rear_right': robot.getDevice('m2_motor')  # CW
}
for motor in motors.values():
    motor.setPosition(float('inf'))
    motor.setVelocity(0.0)

robot.step(timestep)
print("[Info] Controller YAW SIGNS CORRECTED. Final stability test.")
last_print_time = robot.getTime()

# --- 4. MAIN CONTROL LOOP ---
while robot.step(timestep) != -1:
    current_time = robot.getTime()
    dt = timestep / 1000.0

    # --- SENSOR READINGS ---
    roll, pitch, _ = imu.getRollPitchYaw()
    altitude = gps.getValues()[2]
    vertical_velocity = gps.getSpeedVector()[2]
    roll_rate, pitch_rate, yaw_rate = gyro.getValues()

    # --- FLIGHT MODE LOGIC ---
    if not is_hovering and altitude >= TARGET_ALTITUDE * TAKEOFF_THRESHOLD:
        print(f"[State Change] Reached {altitude:.2f}m. Engaging Hover mode.")
        is_hovering = True
        altitude_integral = 0.0

    # --- ALTITUDE CONTROLLER ---
    if is_hovering:
        altitude_error = TARGET_ALTITUDE - altitude
        altitude_integral = np.clip(altitude_integral + altitude_error * dt, -0.5, 0.5)
        p_term = KP_ALTITUDE * altitude_error
        i_term = KI_ALTITUDE * altitude_integral
        d_term = -KD_ALTITUDE * vertical_velocity
        vertical_thrust_adjustment = p_term + i_term + d_term
        base_thrust = HOVER_VELOCITY + vertical_thrust_adjustment
    else:  # Takeoff mode
        base_thrust = TAKEOFF_THRUST

    # --- ATTITUDE CONTROLLER ---
    roll_adjustment = KP_ATTITUDE * (0.0 - roll) - KD_ATTITUDE * roll_rate
    pitch_adjustment = KP_ATTITUDE * (0.0 - pitch) - KD_ATTITUDE * pitch_rate
    yaw_adjustment = KP_YAW * (0.0 - yaw_rate)

    # --- MOTOR MIXING (WITH CORRECTED YAW SIGNS) ---
    # To yaw right (positive adjustment): speed up CW motors (FL, RR), slow down CCW (FR, RL)
    motor_vel_fl = base_thrust - pitch_adjustment + roll_adjustment + yaw_adjustment  # Was -yaw
    motor_vel_fr = base_thrust - pitch_adjustment - roll_adjustment - yaw_adjustment  # Was +yaw
    motor_vel_rl = base_thrust + pitch_adjustment + roll_adjustment - yaw_adjustment  # Was +yaw
    motor_vel_rr = base_thrust + pitch_adjustment - roll_adjustment + yaw_adjustment  # Was -yaw

    velocities = [motor_vel_fl, motor_vel_fr, motor_vel_rl, motor_vel_rr]
    velocities_clipped = [np.clip(v, 0, 4000) for v in velocities]

    motors['front_left'].setVelocity(velocities_clipped[0])
    motors['front_right'].setVelocity(velocities_clipped[1])
    motors['rear_left'].setVelocity(velocities_clipped[2])
    motors['rear_right'].setVelocity(velocities_clipped[3])

    # --- Print status every second ---
    if current_time - last_print_time >= 1.0:
        mode = "HOVER" if is_hovering else "TAKEOFF"
        x_pos, y_pos, _ = gps.getValues()
        print(
            f"Mode: {mode} | Alt: {altitude:.2f}m | X-Drift: {x_pos - 1.08:.2f}m | Y-Drift: {y_pos - 4.72:.2f}m | "
            f"V-Vel: {vertical_velocity:.2f}m/s"
        )
        last_print_time = current_time