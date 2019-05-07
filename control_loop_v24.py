"""
Height Control: Control loop
TU Delft Solar Boat Team 2019
Martijn Brummelhuis
14-3-2019
"""
from solarboat_model import calculate_wing_forces_front, calculate_wing_forces_rear
import matplotlib.pyplot as plt
import numpy as np

    
#Control loop characteristics    
K_p = 80000 #Proportional gain
K_i = 0. #Integral gain
K_d = 900000 #Derivative gains

K_p_rear = 100000 #Rear proportional gain
K_i_rear = 0 #
K_d_rear = 8000000

#Try rule K_d = K_p*37.5 dit slaat nergens op
H_ref = 0.8

#Time characteristics
t = 1000 #Time in seconds of the operation
tstep = 0.1 #Timestep
timevector = np.arange(0, t, tstep) #Making a list of the timesteps

#Initial conditions
h_0 = 0.
h_1 = 0.
h_2 = 0.
rpm_maxon_0 = 0.
rpm_maxon_1 = 0.
rpm_maxon_2 = 0.
C_L_0 = 0.42
C_L_1 = 0.42
C_L_2 = 0.475
velocity = 9.1666667
mass_boat = 810.
integral_0 = 0.
integral_1 = 0.
integral_2 = 0.
previous_error_0 = 0.
previous_error_1 = 0.
previous_error_2 = 0.

height_list_0 = []
height_list_1 = []
height_list_2 = []
h_cg_list = []
h_ref_list = []
roll_angle_list = []
pitch_angle_list = []
time = 0.

for i in timevector: #Control loop; for every timestep, reiterate
    time += 1 #count timesteps

    #Port
    error_0 = H_ref - h_0
    proportional_0 = error_0
    integral_0 = integral_0 + error_0 * tstep
    derivative_0 = (error_0 - previous_error_0) / tstep
    previous_error_0 = error_0
    rpm_maxon_0 = K_p * proportional_0 + K_i * integral_0 + K_d * derivative_0
    if rpm_maxon_0 > 6000.:
        rpm_maxon_0 = 6000.
    if rpm_maxon_0 < -6000.:
        rpm_maxon_0 = -6000.

    #Starboard
    error_1 = H_ref - h_1
    proportional_1 = error_1
    integral_1 = integral_1 + error_1 * tstep
    derivative_1 = (error_1 - previous_error_1) / tstep
    previous_error_1 = error_1
    rpm_maxon_1 = K_p * proportional_1 + K_i * integral_1 + K_d * derivative_1
    if rpm_maxon_1 > 6000.:
        rpm_maxon_1 = 6000.
    if rpm_maxon_1 < -6000.:
        rpm_maxon_1 = -6000.

    #Rear
    error_2 = H_ref - h_2
    proportional_2 = error_2
    integral_2 = integral_2 + error_2 * tstep
    derivative_2 = (error_2 - previous_error_2) / tstep
    previous_error_2 = error_2
    rpm_maxon_2 = K_p_rear * proportional_2 + K_i_rear * integral_2 + K_d_rear * derivative_2
    if rpm_maxon_2 > 6000.:
        rpm_maxon_2 = 6000.
    if rpm_maxon_2 < -6000.:
        rpm_maxon_2 = -6000.

    C_L_0, L_0, D_0 = calculate_wing_forces_front(C_L_0, rpm_maxon_0, velocity)
    C_L_1, L_1, D_1 = calculate_wing_forces_front(C_L_1, rpm_maxon_1, velocity)
    C_L_2, L_2, D_2 = calculate_wing_forces_rear(C_L_2, rpm_maxon_2, velocity)

    net_force_0 = L_0 - mass_boat * 9.81 / 3.
    net_force_1 = L_1 - mass_boat * 9.81 / 3.
    net_force_2 = L_2 - mass_boat * 9.81 / 3.
    delta_h_0 = 0.5 * net_force_0/(mass_boat/3.) * tstep ** 2
    delta_h_1 = 0.5 * net_force_1/(mass_boat/3.) * tstep ** 2
    delta_h_2 = 0.5 * net_force_2/(mass_boat/3.) * tstep ** 2
    h_0 = h_0 + delta_h_0
    h_1 = h_1 + delta_h_1
    h_2 = h_2 + delta_h_2

    if time == 2000.:
        h_0 = 0.6
        h_1 = 1.2
    if time == 6000.:
        h_0 = 0.6
        h_1 = 1.
    if time == 8000.:
        h_0 = 1.4

    #collecting data
    roll_angle = np.degrees(np.tan((h_0 - h_1) / 2.)/2.2)
    h_front = (h_0 + h_1)/2.
    h_cg = (2. * h_front + 3. * h_2)/5.

    pitch_angle = np.degrees(np.tan((h_cg - h_2)/3.5))
    height_list_0.append(h_0)
    height_list_1.append(h_1)
    height_list_2.append(h_2)
    h_cg_list.append(h_cg)
    h_ref_list.append(H_ref)
    roll_angle_list.append(roll_angle)
    pitch_angle_list.append(pitch_angle)

#Plotting that shit
plt.figure()
plt.subplot(3,2,1)
plt.plot(timevector,height_list_0)
plt.plot(timevector,h_ref_list)
plt.ylabel('Height in m')
plt.xlabel('Time in s')
plt.subplot(3,2,2)
plt.plot(timevector,height_list_1)
plt.plot(timevector,h_ref_list)
plt.ylabel('Height in m')
plt.xlabel('Time in s')
plt.subplot(3,2,3)
plt.plot(timevector,height_list_2)
plt.plot(timevector,h_ref_list)
plt.ylabel('Height in m')
plt.xlabel('Time in s')
plt.subplot(3,2,4)
plt.plot(timevector,h_cg_list)
plt.plot(timevector,h_ref_list)
plt.ylabel('Height of the CG in m')
plt.xlabel('Time in s')
plt.subplot(3,2,5)
plt.plot(timevector, roll_angle_list)
plt.ylabel('Roll angle in degrees')
plt.xlabel('Time in s')
plt.subplot(3,2,6)
plt.plot(timevector, pitch_angle_list)
plt.ylabel('Pitch angle in degrees')
plt.xlabel('Time in s')
plt.show()
