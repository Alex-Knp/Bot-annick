from math import *

mass = 8
wheel_radius = 0.03
k_phi = 3.962 * 1e-3 * 60 / (2*pi)
R_a = 5.84
K_v = 1/252 * 60 / (2*pi)
J_r = mass / 2 * wheel_radius*wheel_radius/(19*19)
tau_m = J_r / K_v

controller_time_constant = 0.05

Ki = (R_a * K_v) / (k_phi * controller_time_constant)
Kp = tau_m * Ki

print('Ki =',Ki)
print('Kp =',Kp)
print(" ")