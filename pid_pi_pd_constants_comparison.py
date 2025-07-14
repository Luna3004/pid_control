import matplotlib.pyplot as plt
import numpy as np

# --- CONSTANTS ---
mass = 1
spring_const = 10 
damping_coeff = 2

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
for kp in [10, 50, 100]:
    for ki in [50]:
        position = 0
        velocity = 0
        int_error = 0
        prev_error = 0
        position_pi_list = []
        
        for t in time:
            error = setpoint - position
            int_error += error * time_step
            force = pi_control(error, int_error, kp, ki)

            acceleration = (force - damping_coeff * velocity - spring_const * position) / mass
            velocity += acceleration * time_step
            position += velocity * time_step

            position_pi_list.append(position)
            prev_error = error

        plt.plot(time, position_pi_list, label=f"kp={kp}, ki={ki}")

plt.title("PI Control: Position vs Time")
plt.xlabel("Time (s)")
plt.ylabel("Position (m)")
plt.axhline(setpoint, color='red', linestyle='--', label="Setpoint")
plt.legend()
plt.grid(True)
plt.show()
 

     
plt.figure(figsize=(10, 5))
for kp in[10, 50, 100]:
    for kd in [20]:
            position = 0
            velocity = 0
            int_error = 0
            prev_error = 0
            position_pd_list = []
            
    for t in time:
            error = setpoint - position
            deriv_error = (error - prev_error) / time_step
            force = pd_control(error, deriv_error, kp, kd)

            acceleration = (force - damping_coeff * velocity - spring_const * position) / mass
            velocity += acceleration * time_step
            position += velocity * time_step

            position_pd_list.append(position)
            prev_error = error

    plt.plot(time, position_pd_list, label=f"kp={kp}, kd={kd}")

plt.title("PD Control: Position vs Time")
plt.xlabel("Time (s)")
plt.ylabel("Position (m)")
plt.axhline(setpoint, color='red', linestyle='--', label="Setpoint")
plt.legend()
plt.grid(True)
plt.show()

plt.figure(figsize=(10, 5))
for kp in[10, 100]:
    for kd in [20]:
        for ki in [100]:
            position = 0
            velocity = 0
            int_error = 0
            prev_error = 0
            position_pid_list = []
            
            for t in time:
                error = setpoint - position
                int_error += error * time_step
                deriv_error = (error - prev_error) / time_step
                force = pid_control(error, int_error, deriv_error, kp, ki, kd)

                acceleration = (force - damping_coeff * velocity - spring_const * position) / mass
                velocity += acceleration * time_step
                position += velocity * time_step

                position_pid_list.append(position)
                prev_error = error

            plt.plot(time, position_pid_list, label=f"kp={kp}, ki={ki}, kd={kd}")

plt.title("PID Control: Position vs Time")
plt.xlabel("Time (s)")
plt.ylabel("Position (m)")
plt.axhline(setpoint, color='red', linestyle='--', label="Setpoint")
plt.legend()
plt.grid(True)
plt.show()


            

            
            
            
            



