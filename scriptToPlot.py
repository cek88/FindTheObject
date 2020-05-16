import matplotlib.pyplot as plt
import csv
from mpl_toolkits.mplot3d import Axes3D

x_globe = []
y_globe = []
z_globe = []
x_scene = []
y_scene = []
z_scene = []
x_matchingGlobe = []
y_matchingGlobe = []
z_matchingGlobe = []
x_matchingScene = []
y_matchingScene = []
z_matchingScene = []
x_gs = []
y_gs =[]
z_gs = []
chi = []

data = ["Data/globe.txt", "Data/globe-in-scene.txt", "Data/matchingPointsGlobe.txt", "Data/matchingPointsScene.txt", "Data/matchingPointsGlobeInScene.txt", "Data/chi_stats.txt"]

for j in range(len(data)):
    d = data[j]
    f = open(d,'r').readlines()
    for i in range(len(f)):
        row = f[i]
        row = row.split("\n")
        row = row[0].split()
        if(j==0):
        	x_globe.append(float(row[0]))
        	y_globe.append(float(row[1]))
        	z_globe.append(float(row[2]))
        elif(j==1):
        	x_scene.append(float(row[0]))
        	y_scene.append(float(row[1]))
        	z_scene.append(float(row[2]))
        elif(j==2):
        	x_matchingGlobe.append(float(row[0]))
        	y_matchingGlobe.append(float(row[1]))
        	z_matchingGlobe.append(float(row[2]))
        elif(j==3):
        	x_matchingScene.append(float(row[0]))
        	y_matchingScene.append(float(row[1]))
        	z_matchingScene.append(float(row[2]))
        elif(j==4):
        	x_gs.append(float(row[0]))
        	y_gs.append(float(row[1]))
        	z_gs.append(float(row[2]))
        elif(j==5):
        	chi.append(float(row[0]))


fig1 = plt.figure()
ax = fig1.add_subplot(211, projection='3d')
ax.scatter(x_globe, y_globe, z_globe, c = "red")
ax.scatter(x_matchingGlobe, y_matchingGlobe, z_matchingGlobe, c= "green")
ax.set_xlabel('X Globe')
ax.set_ylabel('Y Globe')
ax.set_zlabel('Z Globe')
asc = fig1.add_subplot(212, projection='3d')
asc.scatter(x_scene, y_scene, z_scene, c = "red")
asc.scatter(x_matchingScene, y_matchingScene, z_matchingScene, c= "green")
asc.set_xlabel('X Scene')
asc.set_ylabel('Y Scene')
asc.set_zlabel('Z Scene')
plt.title("globe and scene with matchings")
#plt.show()

fig2 = plt.figure()
asc = fig2.add_subplot(111, projection='3d')
asc.scatter(x_gs, y_gs, z_gs, c= "green", alpha=0.1)
asc.scatter(x_globe, y_globe, z_globe, c = "red")
asc.set_xlabel('X')
asc.set_ylabel('Y')
asc.set_zlabel('Z')
plt.title("object in scene")
#plt.show()

fig3 = plt.figure()
plt.plot(chi)
plt.title("chi values")
plt.show()
