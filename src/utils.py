import math


# Function returns the sign of a number
def sgn(x):
    if x > 0:
        return 1
    elif x < 0:
        return -1
    else:
        return 0


# Function to consider periodicity and find min delta angle
def min_spin(spin_angle):
    if spin_angle > math.pi:
        spin_angle -= 2 * math.pi
    elif spin_angle < -1 * math.pi:
        spin_angle += 2 * math.pi
    return spin_angle


# Converts from Quaternion to yaw
def convert_planar_quaternion_to_phi(quaternion):
    return 2 * math.atan2(quaternion.z, quaternion.w)
