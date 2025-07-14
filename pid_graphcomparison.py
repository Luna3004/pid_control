import matplotlib.pyplot as plt
import numpy as np

# --- Constants ---
mass = 1
spring_const = 10
damping_coeff = 2


ki = 100.0

setpoint = 1

time_step = 0.001
total_time = 5.0
time = np.arange(0, total_time, time_step)


def simulate_kp(kp, time):
    position = 0
    velocity = 0
    int_error = 0
    prev_error = 0
    position_kp_list = []

    kd = 5
    ki = 100
    
    for t in time:
        disturbance = 5 if 1.5 < t < 1.6 else 0
        
        error = setpoint - position
        int_error += error * time_step
        deriv_error = (error - prev_error) / time_step

        force = kp * error + ki * int_error + kd * deriv_error
        acceleration = (force + disturbance - damping_coeff * velocity - spring_const * position) / mass
        
        velocity += acceleration * time_step
        position += velocity * time_step

        position_kp_list.append(position)
        prev_error = error

    return np.array(position_kp_list)  

def simulate_kd(kd, time):
    position = 0
    velocity = 0
    int_error = 0
    prev_error = 0
    position_kd_list = []

    kp = 100
    ki = 100
    
    for t in time:
        disturbance = 5 if 1.5 < t < 1.6 else 0
        
        error = setpoint - position
        int_error += error * time_step
        deriv_error = (error - prev_error) / time_step

        force = kp * error + ki * int_error + kd * deriv_error
        acceleration = (force + disturbance - damping_coeff * velocity - spring_const * position) / mass
        
        velocity += acceleration * time_step
        position += velocity * time_step

        position_kd_list.append(position)
        prev_error = error

    return np.array(position_kd_list)  

def simulate_ki(ki, time):
    position = 0
    velocity = 0
    int_error = 0
    prev_error = 0
    position_ki_list = []

    kp = 100
    kd = 5
    
    for t in time:
        disturbance = 5 if 1.5 < t < 1.6 else 0
        
        error = setpoint - position
        int_error += error * time_step
        deriv_error = (error - prev_error) / time_step

        force = kp * error + ki * int_error + kd * deriv_error
        acceleration = (force + disturbance - damping_coeff * velocity - spring_const * position) / mass
        
        velocity += acceleration * time_step
        position += velocity * time_step

        position_ki_list.append(position)
        prev_error = error

    return np.array(position_ki_list)  

# --- Simulate & Plot Kp Lines ---
plt.figure(figsize=(12, 6))

for kp in [10, 50, 100]:  # Only these 3 Kp values
    position_kp = simulate_kp(kp, time)
    plt.plot(time, position_kp, label=f"Kp = {kp}")

plt.axhline(y=setpoint, color='r', linestyle='--', label='Setpoint')
plt.title("Position vs Time for Kp = 10, 50, 100")
plt.xlabel("Time (s)")
plt.ylabel("Position (m)")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()

# --- Simulate & Plot Kd Lines ---
plt.figure(figsize=(12, 6))

for kd in [5, 10, 20]:  # Only these 3 Kd values
    position_kd = simulate_kd(kd, time)
    plt.plot(time, position_kd, label=f"Kd = {kd}")

plt.axhline(y=setpoint, color='r', linestyle='--', label='Setpoint')
plt.title("Position vs Time for Kd = 10, 50, 100")
plt.xlabel("Time (s)")
plt.ylabel("Position (m)")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()

# --- Simulate & Plot Ki Lines ---
plt.figure(figsize=(12, 6))

for ki in [10, 50, 100]:  # Only these 3 Kd values
    position_ki = simulate_ki(ki, time)
    plt.plot(time, position_ki, label=f"Ki = {ki}")

plt.axhline(y=setpoint, color='r', linestyle='--', label='Setpoint')
plt.title("Position vs Time for Ki = 10, 50, 100")
plt.xlabel("Time (s)")
plt.ylabel("Position (m)")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()


