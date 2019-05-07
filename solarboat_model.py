# -*- coding: utf-8 -*-
"""
Height Control: Control loop plant
TU Delft Solar Boat Team 2019
Martijn Brummelhuis
29-4-2019

Coordinate system:
1. The origin is located in the center of gravity
2. X-axis points along the symmetry axis of the boat from stern to bow
3. Y-axis points to starboard
4. Z-axis pointing downwards (as per right hand rule)

The corresponding control loop can be found in control_loop.py or control_loop_angles.py
"""
from math import *

def calculate_wing_forces_front(C_L, rpm_maxon, velocity):
    GR_maxon = 19 #Maxon internal gear ratio
    GR_mechanical =280 #Gear ratio of the mechanical system
    C_Lalpha = 0.08 * 0.09333333 #Increase C_L per degree AOA
    rho = 1027 #Density of sea water in kg/m3
    s = 0.15
    change_aoa = rpm_maxon / GR_maxon / GR_mechanical / 60.
    C_L = C_L + C_Lalpha * change_aoa
    C_D = C_L/60. #this can be updated by using the actual L/D ratios
    L = C_L * 0.5 * rho * velocity ** 2 * s
    D = C_D * 0.5 * rho * velocity ** 2 * s

    if C_L > 1.3:
        C_L = 1.3
    return C_L, L, D

def calculate_wing_forces_rear(C_L, rpm_maxon, velocity):
    GR_maxon = 246 #Maxon internal gear ratio
    GR_mechanical = 754 #Gear ratio of the mechanical system
    C_Lalpha = 0.08 * 0.09333333 #Increase C_L per degree AOA
    rho = 1027 #Density of sea water in kg/m3
    s = 0.13
    change_aoa = rpm_maxon / GR_maxon / GR_mechanical / 60.
    C_L = C_L + C_Lalpha * change_aoa
    C_D = C_L/60.
    L = C_L * 0.5 * rho * velocity ** 2 * s
    D = C_D * 0.5 * rho * velocity ** 2 * s
    if C_L > 1.3:
        C_L = 1.3
    return C_L, L, D
    
def calculate_pitch_angle(L_left, L_right, L_rear, tstep, pitch_angle):
    d_cg_front = 1.5 #distance from CG of the front wings in m
    d_cg_rear = 3 # distance from CG of the rear wings in m
    mmoi_y = 370 #Mass moment of inertia bout the y axis in kgm2
    
    pitch_moment = L_left * d_cg_front + L_right * d_cg_front - L_rear * d_cg_rear
    pitch_acceleration = pitch_moment / mmoi_y
    delta_theta = 0.5 * pitch_acceleration * tstep ** 2
    pitch_angle = degrees(pitch_angle + delta_theta)

    return pitch_angle
    
def calculate_roll_angle(L_right, L_left, tstep, roll_angle):
    d_cg_lateral = 2.2 #lateral distance of wings to CG in m
    
    mmoi_x = 225. #Mass moment of inertia about the x axis in kgm2
    
    roll_moment = (L_right - L_left)*d_cg_lateral
    roll_acceleration = roll_moment / mmoi_x
    delta_roll_angle = 0.5* roll_acceleration * tstep ** 2
    roll_angle = degrees(roll_angle + delta_roll_angle)

    return roll_angle

def calculate_motor_thrust(throttle):
    thrust = throttle * 4000.
    return thrust

def determine_height_atcg(h_cg, L_right, L_left, L_rear, mass_boat, tstep):
    sum_forces_upwards = L_right + L_left + L_rear - mass_boat * 9.81
    acceleration_upwards = sum_forces_upwards / mass_boat
    delta_height = 0.5 * acceleration_upwards * tstep ** 2
    h_cg = h_cg + delta_height

    return h_cg

def determine_velocity(D_right, D_left, D_rear, thrust, velocity, tstep, mass_boat):
    D_static = 1 #Static drag in N
    sum_forces_long = thrust - D_right - D_left - D_rear - D_static
    acceleration_long = sum_forces_long / mass_boat
    delta_velocity = acceleration_long * tstep
    velocity = velocity + delta_velocity

    return velocity


def determine_height_atwings(pitch_angle, roll_angle, flying_height):
    d_cg_front = 1 #distance between CG and front wings over x axis
    d_cg_rear = -2 #distance between CG and rear wing over x axis
    d_cg_lateral = 3 #distance between CG and front wings over y axis
    
    #Lots of geometry
    d_abs_front = sqrt(d_cg_front ** 2 + d_cg_lateral ** 2)
    d_ort_front_x = cos(radians(pitch_angle))*d_cg_front
    d_ort_front_y = cos(radians(roll_angle)) * d_cg_lateral
    d_ort_abs = sqrt(d_ort_front_x **2 + d_ort_front_y**2)
    
    #Calculating output (measured heights at wings)
    h_front_starboard = flying_height - sqrt(d_abs_front ** 2 - d_ort_abs ** 2)
    
    h_front_port = flying_height + sqrt(d_abs_front ** 2 - d_ort_abs ** 2)
    
    h_rear = flying_height - d_cg_rear*sin(radians(pitch_angle))
    
    return h_front_starboard, h_front_port, h_rear

def determine_height_atsenix(pitch_angle, roll_angle, flying_height):
    d_cg_middle = 6 #distance between CG and middle senix over x axis
    d_cg_longitudinal = 2 #distance between CG and SH senix over x axis
    d_cg_lateral = 3 #distance between CG and SH senix over y axis
    
    #Lots of geometry
    d_abs_shsenix = sqrt(d_cg_longitudinal ** 2 + d_cg_lateral ** 2)
    d_ort_shsenix_x = cos(radians(pitch_angle)) * d_cg_longitudinal
    d_ort_shsenix_y = cos(radians(roll_angle)) * d_cg_lateral
    d_ort_abs = sqrt(d_ort_shsenix_y ** 2 + d_ort_shsenix_x ** 2)
    
    #Calculating height at senix locations
    h_senix_mid = flying_height + sin(radians(pitch_angle))*d_cg_middle
    
    h_senix_starboard = flying_height + sqrt(d_abs_shsenix ** 2 - d_ort_abs ** 2)
    
    h_senix_port = flying_height - sqrt(d_abs_shsenix ** 2 - d_ort_abs ** 2)
    
    return h_senix_mid, h_senix_starboard, h_senix_port

def solarboat_plant_angles(C_L_right, C_L_left, C_L_rear, rpm_maxon_right, rpm_maxon_left, rpm_maxon_rear, pitch_angle, roll_angle, velocity, h_cg, tstep, throttle, mass_boat):
    C_L_right, L_right, D_right = calculate_wing_forces_front(C_L_right, rpm_maxon_right, velocity)
    C_L_left, L_left, D_left = calculate_wing_forces_front(C_L_left, rpm_maxon_left, velocity)
    C_L_rear, L_rear, D_rear = calculate_wing_forces_rear(C_L_rear, rpm_maxon_rear, velocity)

    thrust = calculate_motor_thrust(throttle)

    pitch_angle = calculate_pitch_angle(L_right, L_left, L_rear, tstep, pitch_angle)
    roll_angle = calculate_roll_angle(L_right, L_left, tstep, roll_angle)

    h_cg = determine_height_atcg(h_cg, L_right, L_left, L_rear, mass_boat, tstep)
    velocity = determine_velocity(D_right, D_left, D_rear, thrust, velocity, tstep, mass_boat)
    return C_L_right, C_L_left, C_L_rear, pitch_angle, roll_angle, h_cg, velocity
