import matplotlib.pyplot as plt
import numpy as np

# --- CONSTANTS ---
mass = 1
spring_const = 10 
damping_coeff = 2

kp = 80
ki = 100
kd = 5

setpoint = 1

time_step = 0.001
total_time = 5.0
time = np.arange(0, total_time, time_step)


def pi_control(error, int_error, kp, ki):

    return kp * error + ki * int_error


def pd_control(error, deriv_error, kp, kd):
    
    return kp * error + kd * deriv_error


def pid_control(error, int_error, deriv_error, kp, ki, kd):
    
    return kp * error + ki * int_error + kd * deriv_error

plt.figure(figsize=(10, 5))

position_pi = 0
velocity_pi = 0
position_pd = 0
velocity_pd = 0
position_pid = 0
velocity_pid = 0
int_error_pi = 0
int_error_pid = 0
deriv_error_pd = 0 
derive_error_pid = 0 
prev_error_pi = 0
prev_error_pd = 0
prev_error_pid = 0
position_pi_list = []
position_pd_list = []
position_pid_list = []


for t in time:
    error_pi = setpoint - position_pi
    error_pd = setpoint - position_pd
    error_pid = setpoint - position_pid
    
    int_error_pi += error_pi * time_step
    int_error_pid += error_pid * time_step
    
    deriv_error_pd = (error_pd - prev_error_pd) / time_step
    deriv_error_pid = (error_pid - prev_error_pid) / time_step


    force_pi = pi_control(error_pi, int_error_pi, kp, ki)
    force_pd = pd_control(error_pd, deriv_error_pd, kp, kd)
    force_pid = pid_control(error_pid, int_error_pid, deriv_error_pid, kp, ki, kd)
    
    acceleration_pi = (force_pi - damping_coeff * velocity_pi - spring_const * position_pi) / mass
    acceleration_pd = (force_pd - damping_coeff * velocity_pd - spring_const * position_pd) / mass
    acceleration_pid = (force_pid - damping_coeff * velocity_pid - spring_const * position_pid) / mass

    velocity_pi += acceleration_pi * time_step
    velocity_pd += acceleration_pd * time_step
    velocity_pid += acceleration_pid * time_step
    
    position_pi += velocity_pi * time_step
    position_pd += velocity_pd * time_step
    position_pid += velocity_pid * time_step


    position_pi_list.append(position_pi)
    position_pd_list.append(position_pd)
    position_pid_list.append(position_pid)
    
    prev_error_pi = error_pi
    prev_error_pd = error_pd
    prev_error_pid = error_pid


plt.plot(time, position_pi_list, label=f"PI: kp={kp}, ki={ki}")
plt.plot(time, position_pd_list, label=f"PD: kp={kp}, kd={kd}")
plt.plot(time, position_pid_list, label=f"PID: kp={kp}, ki={ki}, kd={kd}")

plt.title("PI vs PD vs PID Control: Position vs Time")
plt.xlabel("Time (s)")
plt.ylabel("Position (m)")
plt.axhline(setpoint, color='red', linestyle='--', label="Setpoint")
plt.legend()
plt.grid(True)
plt.show()
 

            

            
            
            
            



