from pyRobotics import *
import matplotlib.pyplot as plt
import numpy as np

#################### TIEMPO ###################

tf = 3 # tiempo de simulacion
ts = 0.1 #  tiempo de muestreo
t = np.arange(0,tf+ts,ts) # vector tiempo

N = len(t) # cantidad de muestras


################### CONDICIONES INICIALES ###################
# posiciones traslacionales
hx = np.zeros(N+1) 
hy= np.zeros(N+1)
hz= np.zeros(N+1)

hx[0]=0
hy[0]=0
hz[0]=0

# posiciones angulares
phi = np.zeros(N+1) 
thetha = np.zeros(N+1)
psi = np.zeros(N+1)

phi[0]=0
thetha[0]=0
psi[0]=0

# velocidades traslacionales
hxp = np.zeros(N+1) 
hyp = np.zeros(N+1)
hzp = np.zeros(N+1)

hxp[0]=0
hyp[0]=0
hzp[0]=0

# velocidades rotacionales
phip = np.zeros(N+1) 
thethap = np.zeros(N+1)
psip = np.zeros(N+1)


phip[0]=0
thethap[0]=0
psip[0]=0

################## PARAMETROS DEL ROBOT ##############

g=9.81 # gravedad [m/s^2]
m = 0.429 # masa total del cuadricoptero [Kg]
Ix = 2.24e-3 # Inercia en x [Nms^2]
Iy = 2.98e-3 # Inercia en y [Nms^2]
Iz = 4.8e-3 # Inercia en z [Nms^2]
l = 0.1785 # Distancia del centro de masa al centro de la helice [m]
d = 2.423e-7 # factor de arrastre [Nms^2]
b = 8.048e-6 # factor de empuje [Ns^2]


######################## ENTRADAS DEL SISTEMA ######################
U1 = 5*np.ones(N) # Fuerza de empuje [N]
U2 = 0.0002*np.ones(N)# Torque en roll [N/m]
U3 = 0*np.ones(N) # Torque en pitch [N/m]
U4 = -0.003*np.ones(N) # Torque en yaw [N/m]


################### BUCLE ####################
for k in range(N):

    hxpp = (np.cos(psi[k])*np.sin(thetha[k])*np.cos(phi[k])+np.sin(psi[k])*np.sin(phi[k]))*(U1[k]/m)
    hypp = (np.sin(psi[k])*np.sin(thetha[k])*np.cos(phi[k])-np.cos(psi[k])*np.sin(phi[k]))*(U1[k]/m)
    hzpp = (np.cos(thetha[k])*np.cos(phi[k]))*(U1[k]/m)-g

    phipp = ((Iy-Iz)*thethap[k]*psip[k])/Ix + (U2[k]/Ix) + thethap[k]*psip[k]
    thethapp = ((Iz-Ix)*psip[k]*phip[k])/Iy + (U3[k]/Iy) - psip[k]*phip[k]
    psipp = ((Ix-Iy)*thethap[k]*phip[k])/Iz + (U4[k]/Iz) + thethap[k]*phip[k]


    # Integracion numerica
    hxp[k+1]=hxp[k]+ts*hxpp
    hyp[k+1]=hyp[k]+ts*hypp
    hzp[k+1]=hzp[k]+ts*hzpp
    
    phip[k+1]=phip[k]+ts*phipp
    thethap[k+1]=thethap[k]+ts*thethapp
    psip[k+1]=psip[k]+ts*psipp

    # Integracion numerica
    
    hx[k+1]=hx[k]+ts*hxp[k+1]
    hy[k+1]=hy[k]+ts*hyp[k+1]
    hz[k+1]=hz[k]+ts*hzp[k+1]
    
    phi[k+1]=phi[k]+ts*phip[k+1]
    thetha[k+1]=thetha[k]+ts*thethap[k+1]
    psi[k+1]=psi[k]+ts*psip[k+1]
    
################### SIMULACION VIRTUAL #################### 
path = "4+Simulacion+del+modelo+dinamico+Fuerzas+y+torques/stl"
color = ["#333333","silver","silver","yellow"]
uav = robotics(path,color)

xmin = -5
xmax = 5
ymin = -5
ymax = 5
zmin = 0
zmax = 10
bounds = [xmin, xmax, ymin, ymax, zmin, zmax]
uav.configureScene(bounds)

uav.initTrajectory(hx,hy,hz)

escala = 2
uav.initRobot(hx,hy,hz,phi,thetha,psi,escala)

uav.startSimulation(1,ts)










