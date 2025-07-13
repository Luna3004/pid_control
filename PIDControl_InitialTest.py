import matplotlib.pyplot as plt

# --- CONSTANTS ---
mass = 1; # kg 
spring_const = 10 # N/m
damping_coeff = 2 # Ns/m


time_step = 0.001 # s
total_time = 5.0 # s

kp = 100; # proportional
ki = 100.0; # integral
kd = 5.0 # derivative 

num_steps = int(total_time/time_step)
print (num_steps)

position = 0
velocity = 0 
acceleration =0 

error = 0 
int_error = 0 
prev_error = 0 

setpoint = 1

time_list = []
position_list = []
velocity_list = []
force_list = []
error_list = []

for step in range(num_steps):
    current_time = step * time_step
    
    error = setpoint - position
    int_error += error * time_step
    deriv_error = (error - prev_error)/time_step
    
    force = kp * error + ki * int_error + kd * deriv_error
    a = (force - damping_coeff * velocity - spring_const*position) /mass
    
    acceleration = a
    velocity += acceleration * time_step
    position += velocity * time_step
    
    time_list.append(current_time)
    position_list.append(position)
    velocity_list.append(velocity)
    force_list.append(force)
    error_list.append(error)
    
    prev_error = error
    

plt.figure(figsize=(10,5))
plt.plot(time_list, position_list, label = "Position")
plt.axhline(y=setpoint, color ='r', linestyle='--', label="Setpoint")
plt.title("Position vs Time")
plt.xlabel("Time (s)")
plt.ylabel("Position (m)")
plt.grid(True)
plt.legend()
plt.show()


plt.figure(figsize=(10,5))
plt.plot(time_list, force_list, color = 'orange', label = "Control Force")
plt.title("Control Force vs Time")
plt.xlabel("Time (s)")
plt.ylabel("Force (N)")
plt.grid(True)
plt.legend()
plt.show()

plt.figure(figsize=(10,5))
plt.plot(time_list, error_list, color = 'green', label = "Error")
plt.title("Error vs Time")
plt.xlabel("Time (s)")
plt.ylabel("Error (m)")
plt.grid(True)
plt.legend()
plt.show()

