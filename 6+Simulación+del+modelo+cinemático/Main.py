from pyRobotics import *
import numpy as np

######################### TIEMPO ########################
tf = 40 # tiempo de simulacion
ts = 0.1 # tiempo de muestreo
t = np.arange(0,tf+ts,ts) # vector tiempo
N = len(t) # Cantidad de muestras

########################## PARAMETROS DEL ROBOT ###############
a = 0.3 #altura del robot en metros [m]

###########################CONDICIONES INICIALES #################
# Centro de masa
x1 = np.zeros(N+1)
y1 = np.zeros(N+1)
z1 = np.zeros(N+1)

x1[0] = 0 #Posicion inicial en el eje x en metros [m]
y1[0] = 0 #Posicion inicial en el eje y en metros [m]
z1[0] = 0 #Posicion inicial en el eje z en metros [m]

phi = np.zeros(N+1)
thetha = np.zeros(N+1)
psi = np.zeros(N+1)

psi[0] = 0*(np.pi/180)

# Punto de control
hx = np.zeros(N+1)
hy = np.zeros(N+1)
hz = np.zeros(N+1)

hx[0] = x1[0] #Posicion inicial en el eje x en metros [m]
hy[0] = y1[0] #Posicion inicial en el eje y en metros [m]
hz[0] = z1[0]+a #Posicion inicial en el eje z en metros [m]


################# VELOCIDADES DE REFERENCIA ######################
uf = 0.5*np.ones(N) # Velocidad lineal en metros/segundos [m/s] eje x
ul = 0.5*np.ones(N) # Velocidad lineal en metros/segundos [m/s] eje y
uz = 0.4*np.ones(N) # Velocidad lineal en metros/segundos [m/s] eje z
w = 3*np.ones(N) # Velocidad angular en rad/segundos [rad/s]

#################### BUCLE DE SIMULACION ######################

for k in range(N):

     # Integral numerica
     psi[k+1] = psi[k] +ts*w[k]

     # Modelo cinematico
     hxp = uf[k]*np.cos(psi[k+1])-ul[k]*np.sin(psi[k+1])
     hyp = uf[k]*np.sin(psi[k+1])+ul[k]*np.cos(psi[k+1])
     hzp = uz[k]

     
     x1[k+1] = x1[k]+ts*hxp
     y1[k+1] = y1[k]+ts*hyp
     z1[k+1] = z1[k]+ts*hzp

     hx[k+1] = x1[k+1]
     hy[k+1] = y1[k+1]
     hz[k+1] = z1[k+1]+a

############################ SIMULACION 3D #############################

# Lectura de partes del robot
path = "/home/carlos/robotMed/drone/control_posicion/stl"
color = ["#333333","red","silver","yellow"]

uav = robotics(path,color)

# Configuracion de la escena
xmin = -3
xmax = 3
ymin = -3
ymax = 3
zmin = 0
zmax = 3
bounds = [xmin,xmax,ymin,ymax,zmin,zmax]
uav.configureScene(bounds)


# Inicializar las trayectorias
uav.initTrajectory(hx,hy,hz)

# Exportar el robot

escala = 2
uav.initRobot(x1,y1,z1,phi,thetha,psi,escala)

# Iniciamos la simulacion
uav.startSimulation(1,ts)






