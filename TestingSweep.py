
import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt
import trimesh
from trimesh import viewer
from trimesh.viewer import windowed
from trimesh import creation, transformations
from shapely.geometry import Point
import math
import mapbox_earcut
pi = math.pi
cos = math.cos
sin = math.sin
x_0 = (300, -15, 10, 0, 0, 0) #x:300, y:-15, z:10, theta:0, yaw: 0, v:0

def dXdt(X,t):
    x,y,z,yaw,theta,v = X
    omega = -pi/12
    alpha = pi/6
    acc = 0.4

    return [v*cos(yaw)*cos(theta),
            v*sin(yaw)*cos(theta),
            v*sin(theta),
            omega,
            alpha,
            acc]

t = np.linspace(0,10,20)
sol = odeint(dXdt,x_0,t)
x,y,z,yaw,theta,v = sol.T
print((x[-1],y[-1],z[-1],yaw[-1],theta[-1],v[-1]))


circle = Point(x[0],z[0]).buffer(0.5)
xyzPath = zip(x,y,z)
scene = trimesh.Scene()
for [x,y,z] in list(xyzPath):
    sphere = trimesh.creation.icosphere(radius=1,color=(0,0,255))
    sphere.apply_translation([x,y,z])
    scene.add_geometry(sphere)
#trajectorySweep = trimesh.creation.sweep_polygon(circle, list(xyzPath))
#scene.add_geometry(trajectorySweep)
scene.show()

ax = plt.axes(projection='3d')
ax.plot3D(x,y,z)
plt.show()
