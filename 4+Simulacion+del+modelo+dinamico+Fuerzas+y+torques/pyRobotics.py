from threading import Thread
import time
import pyvista as pv
import numpy as np
import glob

class robotics:

    def __init__(self,path="",color = None):

        self.color = color
        self.path = path
        filenames = glob.glob(self.path+"/*.stl")
        self.robot = []
        self.robotCopy = []
        self.isTrajectory = False
        
        for filename in filenames:
            self.robot.append(pv.PolyData(filename))
            self.robotCopy.append(pv.PolyData(filename))

    def configureScene(self,bounds, window_size =[1024, 768], title="Python Robotics"):
        self.bounds = bounds
        self.plotter = pv.BackgroundPlotter(window_size=window_size,title=title)
        self.plotter.set_background(color='white')
    
    def initRobot(self,x1,y1,z1,phi,thetha,psi,escala):
          
        self.x1 = x1
        self.y1=  y1
        self.z1=  z1
        self.phi = phi
        self.thetha = thetha
        self.psi = psi

        self.escala = escala
        
        for i in range(len(self.robot)):
            self.robot[i].points *= self.escala
            self.robotCopy[i].points *= self.escala
            if self.color == None:
                self.plotter.add_mesh(self.robotCopy[i],'black')
            else:
                self.plotter.add_mesh(self.robotCopy[i],self.color[i])

          
    def initTrajectory(self,hx,hy,hz):
        
        self.isTrajectory = True
        self.hx = hx
        self.hy = hy
        self.hz = hz
        self.sizeh = len(self.hx)
        points = np.column_stack((np.zeros(self.sizeh),np.zeros(self.sizeh),np.zeros(self.sizeh)))
        self.spline = pv.Spline(points,self.sizeh)
        self.plotter.add_mesh(self.spline,color='red',line_width = 4)
          

    def plotDesiredTrajectory(self,hxd,hyd,hzd):

        sizehd = len(hxd)
        points = np.column_stack((hxd,hyd,hzd))
        self.spline1 = pv.Spline(points,sizehd)
        self.plotter.add_mesh(self.spline1,color='blue',line_width = 4)
        
        
    def startSimulation(self,step = 1,ts=0.1):
        
        cpos = [(-8, -8, 8), # zoom x y z
        (0.5, 0.5, 0.5), # Movimiento x y z
        (0.28, 0.28, 0.28)]
        self.plotter.show_bounds(grid='True',location = 'outer',color = '#000000',bounds = self.bounds, xlabel = 'x [m]', ylabel = 'y [m]', zlabel = 'z [m]')
        #self.plotter.camera_position = cpos
        self.plotter.view_isometric()
        
        self.step = step
        self.ts = ts
        self.thread = Thread(target=self.simulation)
        self.thread.start()


    def simulation(self):
        for k in range(0,len(self.x1),self.step):
            if self.isTrajectory:
                self.plotTrajectory(self.hx[k],self.hy[k],self.hz[k],k)
            self.robotUniciclo(self.x1[k],self.y1[k],self.z1[k],self.phi[k],self.thetha[k],self.psi[k],k)
            time.sleep(self.ts)
            
            
    def robotUniciclo(self,x1,y1,z1,phi,thetha,psi,k):
        
        Rx = np.array([[1, 0, 0],
                                [0, np.cos(phi), -np.sin(phi)],
                                [0, np.sin(phi), np.cos(phi)]])
        
        Ry = np.array([[np.cos(thetha),0,np.sin(thetha)],
                                [    0         ,     1         ,0],
                                [-np.sin(thetha), 0, np.cos(thetha)]])
                    

        Rz = np.array([[np.cos(psi),-np.sin(psi),0],
                        [np.sin(psi), np.cos(psi),0],
                        [    0         ,     0         ,1]])
                        

        R = Rx@Ry@Rz
        
        for i in range(len(self.robotCopy)):
            self.robotCopy[i].points = (R@self.robot[i].points.transpose()).transpose()
            self.robotCopy[i].translate([x1,y1,z1])
            

    def plotTrajectory(self,hx,hy,hz,k):
        self.spline.points[k:self.sizeh,0] = hx
        self.spline.points[k:self.sizeh,1] = hy
        self.spline.points[k:self.sizeh,2] = hz