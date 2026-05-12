import time
import cv2
import numpy as np
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

MAX_RANGO_LIDAR = 5.0 # metros
RESOLUCION = 0.0121 # metro/pixel mapa - simétrico
# RESOLUCION = 0.00975  # mapa asimétrico
N_PARTICULAS = 800 

def inicializar_particulas(num_particulas, mapa_imagen, resolucion):
    alto_pixeles, ancho_pixeles = mapa_imagen.shape
    particulas = []
    
    while len(particulas) < num_particulas:
        px = np.random.randint(0, ancho_pixeles)
        py = np.random.randint(0, alto_pixeles)
        
        if mapa_imagen[py, px] > 200: # verifica si la partícula está en una pared/obstáculo
            x_metros = px * resolucion
            y_metros = (alto_pixeles - py) * resolucion 
            ''' la y se invierte porque en opencv el punto (0,0) se encuentra en 
            la esquina superior izquierda y si aumentamos la Y, el punto baja en 
            la pantalla'''
            theta = np.random.uniform(0, 2 * np.pi)
            particulas.append([x_metros, y_metros, theta])
            
    return np.array(particulas)

def evaluar_particulas(particulas, mapa_imagen, resolucion, lecturas_lidar):
    alto_pixeles, ancho_pixeles = mapa_imagen.shape
    puntajes = []
    
    for particula in particulas:
        x_p, y_p, theta_p = particula
        
        # conversión de metros a pixeles para obtener la posición de la partícula
        px_robot = int(x_p / resolucion)
        py_robot = int(alto_pixeles - (y_p / resolucion))
        
        # si se cae en pared, 0 puntos
        if not (0 <= px_robot < ancho_pixeles and 0 <= py_robot < alto_pixeles) or mapa_imagen[py_robot, px_robot] < 128:
            puntajes.append(0) 
            continue 

        puntaje_actual = 0
        for distancia, angulo_sensor in lecturas_lidar:
            if distancia <= 0 or distancia > MAX_RANGO_LIDAR: 
                continue
    
            angulo_global = theta_p + angulo_sensor # se transforma el agulo local al global

            # se obtiene la distancia del obstáculo relativa a la partícula
            x_obs = x_p + distancia * np.cos(angulo_global)
            y_obs = y_p + distancia * np.sin(angulo_global)
            
            # se calcula el punto donde se detectó un obstáculo a pixeles
            px = int(x_obs / resolucion)
            # se vuelve a invertir la Y por lo mencionaado de opencv más arriba
            py = int(alto_pixeles - (y_obs / resolucion)) 
            
            if 0 <= px < ancho_pixeles and 0 <= py < alto_pixeles:
                # se revisa el color del mapa en esa posición (0=pared, 255=vacío)
                valor_pixel = mapa_imagen[py, px]

                # si el pixel es negro se suman puntos
                puntos_ganados = 255 - valor_pixel
                puntaje_actual += puntos_ganados
                
        puntajes.append(puntaje_actual)
        
    return np.array(puntajes)

def filtrar_y_remuestrear(particulas, puntajes, num_particulas, mapa_imagen, resolucion):
    # normalización de pesos
    puntajes = np.array(puntajes)
    suma_puntajes = np.sum(puntajes)
    pesos = puntajes / suma_puntajes
    
    num_sobrevivientes = int(num_particulas * 0.95) # solo se queda el 95% de las partículas
    # el 5% restante se queda explorando por si sucede un secuestro
    num_aleatorias = num_particulas - num_sobrevivientes 
    
    indices_elegidos = np.random.choice(range(len(particulas)), size=num_sobrevivientes, p=pesos)
    particulas_sobrevivientes = particulas[indices_elegidos].copy()
    
    # se crea ruido para descentralizar las partículas y que puedan detectar variaciones
    ruido_x = np.random.normal(0, 0.05, num_sobrevivientes)
    ruido_y = np.random.normal(0, 0.05, num_sobrevivientes)
    ruido_theta = np.random.normal(0, 0.1, num_sobrevivientes)
    
    # suma de ruidos
    particulas_sobrevivientes[:, 0] += ruido_x
    particulas_sobrevivientes[:, 1] += ruido_y
    particulas_sobrevivientes[:, 2] += ruido_theta
    
    # inyección de partículas exploradoras (5%)
    particulas_exp = inicializar_particulas(num_aleatorias, mapa_imagen, resolucion)
    
    # unión de partículas
    nuevas_particulas = np.vstack((particulas_sobrevivientes, particulas_exp))
    
    return nuevas_particulas

def leer_fast_hokuyo(sim):
    # se obtienen los datos que publica el sensor en coppelia
    datos_crudos = sim.getStringSignal('measuredDataAtThisTime')
    lecturas = []
    
    if datos_crudos:
        # se divide el texto por las comas y lo convertimos a floats
        datos = [float(x) for x in datos_crudos.split(',') if x]
        
        for i in range(0, len(datos), 3):
            x = datos[i]
            y = datos[i+1]

            # conversión de cartesianas a polares
            distancia = np.sqrt(x**2 + y**2) # que tan lejos está el obstáculo
            angulo = np.arctan2(y, x) # la dirección relativa de dicho obstaculo
            lecturas.append((distancia, angulo))
                
    return lecturas

mapa = cv2.imread('mapa.png', cv2.IMREAD_GRAYSCALE) # descomentar para mapa simétrico
#mapa = cv2.imread('mapa2.png', cv2.IMREAD_GRAYSCALE) 
print("Mapa cargado")

client = RemoteAPIClient()
sim = client.require('sim')
print("Conectado a CoppeliaSim")

robotHandle = sim.getObject('/PioneerP3DX') 
motorIzq = sim.getObject('/PioneerP3DX/leftMotor')
motorDer = sim.getObject('/PioneerP3DX/rightMotor')

particulas = inicializar_particulas(N_PARTICULAS, mapa, RESOLUCION)
print("Partículas inicializadas")

# datos de PioneerP3DX
RADIO_RUEDA = 0.0975 
SEPARACION_RUEDAS = 0.381 
dt = 0.05 

sim.startSimulation()

v_izq = 0.0
v_der = 0.0

# variable de tiempo para mejorar la sincronizacón entre el mapa 2D y la simulación
t_anterior = sim.getSimulationTime() 
try:
    alto_pixeles, ancho_pixeles = mapa.shape

    while True:
            lecturas_lidar = leer_fast_hokuyo(sim)

            # Ase toma solo el 10% de los puntos para que no haya lag
            lecturas_lidar = lecturas_lidar[::10]
        
            vel_izq = sim.getJointVelocity(motorIzq)
            vel_der = sim.getJointVelocity(motorDer)
            
            # se calcula el tiempo exacto
            t_actual = sim.getSimulationTime()
            dt = t_actual - t_anterior
            t_anterior = t_actual
            
            # Si dt es muy chiquito o 0 se salta el cálculo
            if dt <= 0:
                continue
                
            # DEAD RECKONING
            v = RADIO_RUEDA * (vel_der + vel_izq) / 2.0 # velocidad linael
            w = (RADIO_RUEDA / SEPARACION_RUEDAS) * (vel_der - vel_izq) # velocidad angular
            
            # se actualizan las partículas 
            particulas[:, 0] += v * np.cos(particulas[:, 2]) * dt
            particulas[:, 1] += v * np.sin(particulas[:, 2]) * dt
            particulas[:, 2] += w * dt
            
            if len(lecturas_lidar) > 0:
                # se compara lo que cada partícula "ve" con lo que el robot en coppelia reporta
                puntajes = evaluar_particulas(particulas, mapa, RESOLUCION, lecturas_lidar)
                # se aplica el 95% de particulas sobrevivientes y el 5% de exploración
                particulas = filtrar_y_remuestrear(particulas, puntajes, N_PARTICULAS, mapa, RESOLUCION)
            
            # se envían las velocidades a coppelia para moevr el robot
            sim.setJointTargetVelocity(motorIzq, v_izq)
            sim.setJointTargetVelocity(motorDer, v_der)

            # para mostrar mapa 
            mapa_visual = cv2.cvtColor(mapa, cv2.COLOR_GRAY2BGR)
            
            for p in particulas:
                px = int(p[0] / RESOLUCION)
                py = int(alto_pixeles - (p[1] / RESOLUCION))
                
                if 0 <= px < ancho_pixeles and 0 <= py < alto_pixeles:
                    if mapa[py, px] > 128: 
                        cv2.circle(mapa_visual, (px, py), 2, (180, 105, 255), -1) 
            
            cv2.imshow("Montecarlo", mapa_visual)
            
            # control con teclado 
            tecla = cv2.waitKey(1) & 0xFF
            
            if tecla == ord('w'):
                v_izq, v_der = 2.0, 2.0   # adelante
            elif tecla == ord('s'):
                v_izq, v_der = -2.0, -2.0 # reversa
            elif tecla == ord('a'):
                v_izq, v_der = -1.5, 1.5  # girar Izquierda
            elif tecla == ord('d'):
                v_izq, v_der = 1.5, -1.5  # girar Derecha
            elif tecla == 255:            # 255 = ninguna tecla presionada
                v_izq, v_der = 0.0, 0.0   # frenar 
            
            # Avanzar simulación
            client.step()

except KeyboardInterrupt:
    sim.stopSimulation()
    print("\nSimulación detenida")