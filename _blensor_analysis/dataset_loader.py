import numpy as np
import matplotlib.pyplot as plt

def loadscan(sensors,scan, noisy):
        
        pos= np.loadtxt("./_blensor_analysis/BLENSOR/pitt_sensor_positions.csv", delimiter=",",skiprows=1,usecols=range(0,3))
    
        points=[]
        path=str(scan)
        for i in range (0,sensors):
                data11= np.loadtxt("./_blensor_analysis/BLENSOR/sensor_"+str(i)+"_"+path+".csv", delimiter=",",skiprows=1)
                
                a=pos[i,0]
                b=pos[i,1]
                c=pos[i,2]

                if(noisy==1):
                        data1=[]
                        
                        
                        addx=np.full_like(data11[:,8],a)
                        xcoords=data11[:,8]+addx
                        data1.append(xcoords)
                        
                        
                        addy=np.full_like(data11[:,9],b)
                        ycoords=data11[:,9]+addy
                        data1.append(ycoords)
                        
                        
                        addz=np.full_like(data11[:,10],c)
                        zcoords=data11[:,10]+addz
                        data1.append(zcoords)
                        

                else:
                        data1=[]

                        addx=np.full_like(data11[:,5],a)
                        xcoords=data11[:,5]+addx
                        data1.append(xcoords)
                        

                        addy=np.full_like(data11[:,6],b)
                        ycoords=data11[:,6]+addy
                        data1.append(ycoords)
                        
                        addz=np.full_like(data11[:,7],c)
                        zcoords=data11[:,7]+addz
                        data1.append(zcoords)
                        

                points1 = np.vstack(data1).T
                points.append(points1)

        return points

def loadpitt():

        trajectories= np.loadtxt("./_blensor_analysis/BLENSOR/pitt_trajectories.csv", delimiter=",",skiprows=1,usecols=range(0,3))
        x_coords=trajectories[:,1]
        y_coords=trajectories[:,2]
        plt.plot(x_coords, y_coords, marker='o', linestyle='',markersize=3)  
        plt.xlabel("X-axis")
        plt.ylabel("Y-axis")
        plt.title("Real Trajectories")
        plt.show()
                
