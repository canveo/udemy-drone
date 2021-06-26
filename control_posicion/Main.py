from pyRobotics import *
import matplotlib.pyplot as plt
import numpy as np

#################### TIEMPO ###################
tf = 20 # tiempo de simulacion
ts = 0.1 #  tiempo de muestreo
t = np.arange(0,tf+ts,ts) # vector tiempo
N = len(t) # cantidad de muestras

###################  PARAMETROS DEL ROBOT ################### 
a = 0.3  # Altura del robot en metros [m]
b = 0.1 # Distancia hacia el punto de control en metros [m]

################### CONDICIONES INICIALES ###################
# Centro de masa
x1 = np.zeros(N+1) # Inicializar variables 
y1 = np.zeros(N+1)
z1 = np.zeros(N+1)


x1[0] =  -3   # Posicion inicial en el eje x en metros [m]
y1[0] =  -2  # Posicion inicial en el eje y en metros [m]
z1[0] =  0   # Posicion inicial en el eje z en metros [m]


phi = np.zeros(N+1)
thetha = np.zeros(N+1)
psi = np.zeros(N+1)

psi[0] = 0*(np.pi/180)  # Orientacion inicial en radianes [rad]



# Punto de control
hx = np.zeros(N+1) # Inicializar variables 
hy = np.zeros(N+1)
hz = np.zeros(N+1)

# Cinematica directa
hx[0] = x1[0]+b*np.cos(psi[0])
hy[0] = y1[0]+b*np.sin(psi[0])
hz[0] = z1[0]+a

######################### POSICION DESEADA ##################
hxd = 3
hyd = -2.5
hzd = 3

psid = 90*(np.pi/180)


################### VELOCIDADES DE REFERENCIA #################### 

ufRef = np.zeros(N) # Velocidad lineal en metros/segundos [m/s] eje x 
ulRef = np.zeros(N) # Velocidad lineal en metros/segundos [m/s] eje y
uzRef = np.zeros(N) # Velocidad lineal en metros/segundos [m/s] eje z
wRef = np.zeros(N) # Velocidad angular en radianes/segundos [rad/s]

################### ERRORES ####################
hxe = np.zeros(N) # Error en el eje x en metros [m]
hye = np.zeros(N)  # Error en el eje y en metros [m]
hze = np.zeros(N) # Error en el eje z en metros [m]
psie = np.zeros(N) # Error de orientación en radianes [rad]


################### BUCLE ####################
for k in range(N):

     ###################### CONTROLADOR ######################

     # Errores

     hxe[k] = hxd-hx[k]
     hye[k] = hyd-hy[k]
     hze[k] = hzd-hz[k]
     psie[k] = psid-psi[k]

     he = np.array([[hxe[k]],[hye[k]],[hze[k]],[psie[k]]]) # vector de errores (4x1)

     # Matriz Jacobiana

     J = np.array([[np.cos(psi[k]), -np.sin(psi[k]), 0  , -b*np.sin(psi[k])],
                   [np.sin(psi[k]),  np.cos(psi[k]), 0  , b*np.cos(psi[k]) ],
                   [0             ,  0             , 1  , 0                ],
                   [0             ,  0             , 0  ,                 1]])

     # Parametros de control

     K = 3*np.array([[1,0,0,0],
                     [0,1,0,0],
                     [0,0,1,0],
                     [0,0,0,1]])
     
     # Ley de control

     qpRef = np.linalg.pinv(J)@K@he # velocidades de referencia

     ufRef[k] = qpRef[0][0]
     ulRef[k] = qpRef[1][0]
     uzRef[k] = qpRef[2][0]
     wRef[k] = qpRef[3][0]

     
     #################### APLICAR ACCIONES DE CONTROL #####################

    # Integral numerica
     psi[k+1] = psi[k]+ts*wRef[k]

     # Modelo cinemático
     x1p = ufRef[k]*np.cos(psi[k+1])-ulRef[k]*np.sin(psi[k+1])
     y1p = ufRef[k]*np.sin(psi[k+1])+ulRef[k]*np.cos(psi[k+1])
     z1p = uzRef[k]

     # Integral numerica
     x1[k+1] = x1[k] + ts*x1p
     y1[k+1] = y1[k] + ts*y1p
     z1[k+1] = z1[k] + ts*z1p

     # Cinematica directa
     hx[k+1] = x1[k+1]+b*np.cos(psi[k+1])  
     hy[k+1] = y1[k+1]+b*np.sin(psi[k+1])
     hz[k+1] = z1[k+1]+a

     
################### SIMULACION VIRTUAL #################### 
# Lectura de partes del robot
path = "/home/carlos/robotMed/drone/control_posicion/stl"
color = ["#333333","red","silver","yellow"]
uav = robotics(path,color)

# Configuracion de la escena
xmin = -5
xmax = 5
ymin = -5
ymax = 5
zmin = 0
zmax = 5
bounds = [xmin, xmax, ymin, ymax, zmin, zmax]
uav.configureScene(bounds)

# Inicializar las trayectorias
uav.initTrajectory(hx,hy,hz)

# Exportar el robot
escala = 2
uav.initRobot(x1,y1,z1,phi,thetha,psi,escala)

# Iniciamos la simulacion
uav.startSimulation(1,ts)
     
############################## Graficas ######################
# Errores
fig = plt.figure()

plt.subplot(211)
plt.plot(t,hxe,'b', linewidth = 2, label = 'hxe')
plt.plot(t,hye,'r', linewidth = 2, label = 'hye')
plt.plot(t,hze,'k', linewidth = 2, label = 'hze')
plt.legend(loc = 'upper right')
plt.xlabel("Tiempo [s]")
plt.ylabel("Error [m]")
plt.grid()

plt.subplot(212)
plt.plot(t,psie,'b', linewidth = 2, label = 'psie')
plt.legend(loc = 'upper right')
plt.xlabel("Tiempo [s]")
plt.ylabel("Error [m]")
plt.grid()

# Acciones de control
fig = plt.figure()

plt.subplot(211)
plt.plot(t,ufRef,'b', linewidth = 2, label = 'ufRef')
plt.plot(t,ulRef,'r', linewidth = 2, label = 'ulRef')
plt.plot(t,uzRef,'k', linewidth = 2, label = 'uzref')
plt.legend(loc = 'upper right')
plt.xlabel("Tiempo [s]")
plt.ylabel("Velocidades lineales [m/s]")
plt.grid()

plt.subplot(212)
plt.plot(t,wRef,'b', linewidth = 2, label = 'wRef')
plt.legend(loc = 'upper right')
plt.xlabel("Tiempo [s]")
plt.ylabel("Velocidad angular [rad/s]")
plt.grid()

plt.show()




















