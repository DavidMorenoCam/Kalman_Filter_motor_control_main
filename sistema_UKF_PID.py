import os                       
import time                   
os.system ("sudo pigpiod")      
time.sleep(1)                   
import pigpio                   
import FaBo9Axis_MPU9250        
import json
import matplotlib.pyplot as  plt
import numpy as np
from math import *
import model
import sys
import select
import tty
import termios

from model import pendulo
from filterpy.kalman import UnscentedKalmanFilter as UKF
from filterpy.kalman import unscented_transform, MerweScaledSigmaPoints
from filterpy.common import Q_discrete_white_noise

error_ant=0
error_integral =0
Km = 0.004198202773919557

def isData():
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

def pid(error,T):
    global error_integral
    global error_ant
    kp =  300
    ki = 100
    kd = 60

    # Proporcional
    up = error*kp

    # Integral
    error_integral += error*T
    ui = error_integral*ki

    # derivativo
    error_derivativo = (error- error_ant)/T
    error_ant = error
    ud = kd * error_derivativo

    # Accion de control
    u = up + ui + ud
    print(u)
    if u < 0:
        u = 0
    if u > 100:
        u = 99
    return u

#This is the auto calibration procedure of a normal ESC
def calibrate():                                            
    ESC=18                                                  
    pig = pigpio.pi()                                       
    max_value = 2100                                        
    min_value = 1000                                        
    print("Frecuencia de PWM: " + str(pig.get_PWM_frequency(ESC)))
    pig.set_servo_pulsewidth(ESC, 0)   
    print("Disconnect the battery and press Enter")
    inp = input()
    if inp == '':
        pig.set_servo_pulsewidth(ESC, max_value)
        print("Connect the battery NOW.. you will here two beeps, then wait for a gradual falling tone then press Enter")
        inp = input()
        if inp == '':            
            pig.set_servo_pulsewidth(ESC, min_value)
            print("Wierd eh! Special tone")
            time.sleep(7)
            print("Wait for it ....")
            time.sleep(5)
            print("Im working on it, DONT WORRY JUST WAIT.....")
            pig.set_servo_pulsewidth(ESC, 0)
            time.sleep(2)
            print("Arming ESC now...")
            pig.set_servo_pulsewidth(ESC, min_value)
            time.sleep(1)
            print("See.... uhhhhh")
            
            
def control():
    
    # Variables
    T = 0.04
    T_sim = 40
    error_integral = 0
    tiempo_ant = 0
    ESC=18                                                              
    timer = 0                                                           
    real_ang = 0
    ang_input = 0
    pwm_ideal = 0
    
    # listas
    tiempo, error_angulo_list, control_list, ref_angle_list= [0.0], [0.0], [0.0], [0.0]  
    sim_angle_list, sim_speed_list, sim_acc_list = [0.0], [0.0], [0.0]         
    accel_X, accel_Y, accel_Z = [0.0], [0.0], [0.0]                     
    real_speed_list,  gyro_Y,  gyro_Z, real_ang_list  = [0.0], [0.0], [0.0], [0.0]  
    filtered_angle_list, filtered_speed_list = [0.0], [0.0]    


    # Kalman filter initialization
    sigmas = MerweScaledSigmaPoints(2, alpha=0.3, beta=2., kappa=1.)
    ukf = UKF(dim_x=2, dim_z=2, fx=model.pendulo, hx=model.h_cv, dt=T, points=sigmas, residual_x=model.residual_x, residual_z=model.residual_h)
    ukf.P = np.diag([1, 1])
    ukf.x = [0, 0]
    Q_value = 1
    ukf.Q = np.diag([Q_value/100000000, Q_value])
    beta = 2
    ukf.R = np.diag([beta*1000, beta])


    mpu9250 = FaBo9Axis_MPU9250.MPU9250()                               
    pig = pigpio.pi()                                                   

    print("Motor armado y calibrado. Comienza el funcionamiento del sistema")
    time.sleep(1)                                                       
    print("Introduzca el angulo en grados entre 0 y 60 y pulse enter")
    old_settings = termios.tcgetattr(sys.stdin)
    try:
        while (timer < T_sim):                            
            """
            if isData():
                ang_input = int(input())
                ang_input_anterior = ang_input
                if ang_input > 60 :
                    ang_input = 60"""
            if (timer >= 5 )& (timer<=25):
                ang_input = 20
            if timer > 25:
                ang_input = 0

            timer0 = time.time()                          
            tiempo.append(timer)                          
            timer += T                                    
        

            reference_angle = (ang_input*pi)/180          
            ref_angle_list.append(reference_angle)

            # UKF prediction state
            ukf.predict(dt=T, u=pwm_ideal, Km=Km)
        
            # funcion real
            gyro = mpu9250.readGyro()               
            real_speed_list.append(gyro['x'])       
            real_ang += gyro['x']*T
            real_ang_list.append(real_ang)          

            # UKF update state
            z = [real_ang, gyro['x']]
            ukf.update(z, dt=T)
    
            filtered_angle_list.append(ukf.x[0])                   
            filtered_speed_list.append(ukf.x[1])                   

            error_angulo_list.append(ref_angle_list[-1] - filtered_angle_list[-1])
            
            
            pwm_ideal = pid(error_angulo_list[-1],T)
            control_list.append(pwm_ideal)
            print('ref: ' + str(np.rad2deg(reference_angle)) + ' ideal: ' + str(pwm_ideal) +  '  error: ' + str(error_angulo_list[-1]) + '  angulo filtrado: ' + str(np.rad2deg(filtered_angle_list[-1])) + '  angulo sensor: ' + str(np.rad2deg(real_ang_list[-1])))
            # Adaptacion señal PWM  
            k_pwm = 8.6
            k_offset = 1140
            pwm_real = int(pwm_ideal*k_pwm + k_offset)

            # Sistema real
            pig.set_servo_pulsewidth(ESC, pwm_real)
        
            # Resultados de modelo
            x = np.array([sim_angle_list[-1], sim_speed_list[-1]])
            ang, vel = model.pendulo(x, T, pwm_ideal, Km)
            sim_angle_list.append(ang)                  
            sim_speed_list.append(vel)                   
        
            # Control de tiempo
            timer1 = time.time()                       
            dt = timer1-timer0
            if( dt < T):                            
                time.sleep(T - dt)                  
            else:
                print('Sobrepasado tiempo de control')

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        
    plt.figure(1)
    plt.plot(tiempo,  real_ang_list, tiempo, sim_angle_list, tiempo, filtered_angle_list, tiempo, ref_angle_list)
    plt.legend(('MPU', 'simulacion', 'UKF output', 'angulo_referencia'), prop = {'size':10}, loc = 'upper right')
    plt.xlabel('Tiempo (s)')
    plt.ylabel('Angulo (rad)')
    plt.title('Angulo del sistema')
    plt.grid()

    plt.figure(2)
    plt.plot(tiempo, real_speed_list, tiempo, sim_speed_list, tiempo, filtered_speed_list)
    plt.legend(('MPU', 'Simulacion', 'UKF output'), prop = {'size':10}, loc = 'upper right')
    plt.xlabel('Tiempo (s)')
    plt.ylabel('Velocidad (rad/s)')
    plt.title('Velocidad del sistema')
    plt.grid()

    plt.figure(3)
    plt.plot(tiempo, error_angulo_list, tiempo, control_list )
    plt.legend(('error', 'u'), prop = {'size':10}, loc = 'upper right')
    plt.xlabel('Tiempo (s)')
    plt.ylabel('error')
    plt.title('error')
    plt.grid()

    plt.figure(4)
    plt.plot(tiempo,   real_ang_list, tiempo, filtered_angle_list, tiempo, ref_angle_list)
    plt.legend(('MPU', 'Filtro output', 'angulo_referencia'), prop = {'size':10}, loc = 'upper right')
    plt.xlabel('Tiempo (s)')
    plt.ylabel('Angulo (rad)')
    plt.title('Angulo del sistema')
    plt.grid()
    
    plt.show()

    outfile = open('data_IMU.json', 'w')
    for timer, real_ang, real_speed, model_ang, model_speed in zip(tiempo, real_ang_list, real_speed_list, sim_angle_list, sim_speed_list):
      data = {'Tiempo': timer, 'Angulo_MPU': real_ang, 'Velocidad_MPU': real_speed, 'Angulo_sim': model_ang, 'Velocidad_sim': model_speed}
      json.dump(data,outfile, sort_keys=True)

    outfile.close()
    print("Los datos se han grabado")
            

if __name__ == '__main__':
    #calibrate()
    control()               
            



