import matplotlib.pyplot as plt
from funcion_pendulo import * 
import funcion_pendulo as fp
import numpy as np


# declaracion de variables
pwm = 0
dt = 0.00095                                    
tiempo_final = 60                               
pwm_introducida = 0
tiempo = 0                                       

xdata, y1data, y2data, y3data = [], [], [], []   
pwm_input = []                                   
def entrada_datos():
    global pwm_input
    print("**************************************AVISO***************************************************")
    print("SE DEBEN INTRODUCIR SENALES DE PWM DE 0 A 100 QUE EQUIVALE A PORCENTAJE DE PWM")
    print("***********************************************************************************************")
    print("***********************************************************************************************")
    for x in range(5):
        valor_pwm = float(input("Introzuca 5 valores (entre 0 y 100) de pwm a comparar: "))
        pwm_input.append(valor_pwm)
    print("Las PWM introducidas son: {}".format(pwm_input))
    print("A continuacion se representaran las 5 graficas con una pwm diferente para cada grafica")
    print("Las diferentes PWM se activan en t=2s y se ponen a 0 en t=8s")



    
    
def momento_inercia():
    masa_barra_real = 0.18255       # kg  masa total de la barra sin cilindro de eje
    masa_motor = 0.06205            # kg  masa total del motor mas helice 
    r_cilindro = 0.003              # m   radio del cilindro eje
    h_cilindro = 0.025              # m   longitud transversal del cilindro eje
    barra_dim_x = 0.025             # m   dimensiones eje x barra
    barra_dim_y = 0.025             # m   dimensiones eje y barra
    barra_dim_z = 0.39              # m   dimensiones eje zx barra
    dist_z_Cilindro_CM = 0.184      # m   distancia del CM de la barra sin quitar cilindro al eje del cilindro
    CM_motor_x = 0.026              # m   distancia del CM del motor al CM de la barra en eje x
    CM_motor_z = 0.354              # m   distancia del CM del motor al CM de la barra en eje z
      
    # valores para calcular el momento de inercia
    volumen_barra = barra_dim_x * barra_dim_y * barra_dim_z   # calculo volumen barra sin quitar cilindro
    volumen_cilindro = pi * (r_cilindro ** 2) * h_cilindro    # calculo volumen cilindro
    volumen_barra_real = volumen_barra - volumen_cilindro     # calculo volumen barra eliminando el cilindro
    densidad_madera = masa_barra_real/volumen_barra_real      # calculo densidad de la madera
    masa_cilindro = densidad_madera * volumen_cilindro        # calculo la masa del cilindro
    masa_barra = densidad_madera * volumen_barra              # calculo la masa de la barra sin quitar el cilindro
    
    # distancia del Centro de Masas del motor al eje de giro
    radio_CM_motor = (CM_motor_x ** 2) + (CM_motor_z ** 2)
    
    # distancia del Centro de Masas  de la barra al eje de giro
    # eje coordenadas en el centro de la barra, al ser un figura simetrica
    dist_z_CM_SolidoCompuesto = (dist_z_Cilindro_CM * (-volumen_cilindro))/(volumen_barra-volumen_cilindro)
    dist_z_CM_Eje_giro = (dist_z_Cilindro_CM - dist_z_CM_SolidoCompuesto)

    # calculo de momento de inercia
    momento_inercia_barra = (((1/12) * masa_barra * ((barra_dim_x ** 2) + (barra_dim_z ** 2))) + (masa_barra * (dist_z_CM_Eje_giro ** 2))) 
    momento_inercia_cilindro = (1/2) * masa_cilindro * (r_cilindro ** 2)
    momento_inercia_motor = masa_motor * (radio_CM_motor ** 2)    
    momento_inercia_total = momento_inercia_barra - momento_inercia_cilindro + momento_inercia_motor
    print("Momento Inercia Total en Kg*m2: ", momento_inercia_total)
    print("d: ", dist_z_CM_Eje_giro)
    
def funcion():
    global pwm 
    global tiempo 
    global dt
    global xdata
    global y1data
    global y2data
    global y3data
    global activa_pwm
    accel = 0
    vel = 0
    ang = 0
    vel1 = 0
    accel1 = 0
    ang1 = 0
    indice_pwm = 0

    i = 1
    num_posicion = tiempo_final / dt                  # hallo el numero de posiciones que tendra la lista segun mi tiempo_fina
    while indice_pwm < len(pwm_input):
        activa_pwm = pwm_input[indice_pwm]
        while i <= num_posicion + 1:                  # el indice de la lista en python comienza en 1, por tanto sumo 1 a la posicion final
            xdata.append(tiempo)                      # agrego tiempo actual a la nueva posicion en la lista
            tiempo += dt                              # incremento el tiempo sumando los incrementos dt
            tiempo = round(tiempo, 8)                 # necesito redondear para que me reconozca el tiempo de 2 s y 8 s
            i = i + 1                                 # incremento posicion de la lista
            if tiempo < 2:                            # esto lo he tenido que forzar para que se comporte adecuadamente
                pwm = 0
                vel = 0
                accel = 0
            if pwm <= 6:
                pwm = 0
            vel = vel1
            accel = accel1
            ang = ang1
            
            fp.pendulo(Km, accel, vel, ang, pwm, dt)                      
            if tiempo >= 2.0:                         
                pwm = activa_pwm  
            if tiempo >= 35.0:                         
                pwm = 0                               
            ang_grad1, vel1, accel1, ang1 = fp.pendulo(Km, accel, vel, ang, pwm, dt)

            y1data.append(ang_grad1)                  
            y2data.append(vel1)                        
            y3data.append(accel1)                      
            #vel = vel1
            #accel = accel1
            #ang = ang1

            y1data.append(ang_grad1)                   
            y2data.append(vel1)                        
            y3data.append(accel1)                      
            vel = vel1
            accel = accel1
            ang = ang1

    

        plt.figure(indice_pwm)
        plt.suptitle("PWM: {}".format(activa_pwm)) 
        plt.subplot(311)
        plt.plot( xdata, y1data, '-')
        plt.grid()
        plt.ylabel("Grados")
        plt.title("Angulo en grados") 
        plt.subplot(312)
        plt.plot(xdata, y2data, '-r')
        plt.grid()
        plt.ylabel("rad/s")
        plt.title("Velocidad")
        plt.subplot(313)
        plt.plot(xdata, y3data, '-y')
        plt.grid()
        plt.xlabel("Segundos")
        plt.ylabel("rad/s2")
        plt.title("Aceleracion")
        indice_pwm = indice_pwm + 1
        xdata, y1data, y2data, y3data = [], [], [], []          
        i = 1
        tiempo = 0

    plt.show()
  




if __name__ == '__main__':
    momento_inercia()
    #entrada_datos()
    #funcion()
