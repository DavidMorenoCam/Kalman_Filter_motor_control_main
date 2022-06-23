import numpy as np
from math import tan, sin, cos, sqrt
import utils
from termcolor import colored


def pendulo( x, dt, u, Km):

    b = 0.02
    m = 0.2476
    g = 9.7996412681
    d = 0.184535139296563
    J = 0.009524722263827562
    curr_angulo = x[0]
    curr_velocidad = x[1]
    estimated_acceleracion = ((u*Km - b*curr_velocidad - m*g*d*np.sin(curr_angulo))/J)
    estimated_velocidad = curr_velocidad + estimated_acceleracion*dt                    
    estimated_angulo = curr_angulo +  estimated_velocidad*dt                            
    return (estimated_angulo, estimated_velocidad)
 
def h_cv(x, dt):
    za = x[0] + x[1]*dt
    zw = x[1]
    return [za, zw]

def normalize_angle(x):
    x = x % (2 * np.pi)  # force in range [0, 2 pi)
    if x > np.pi:        # move to [-pi, pi)
      x -= 2 * np.pi
    return x

def residual_h(a, b):
    y = a - b
    y[0] = normalize_angle(y[0])
    return y

def residual_x(a, b):
    y = a - b
    y[0] = normalize_angle(y[0])
    return y
