
import trimesh
from trimesh import viewer
from trimesh.viewer import windowed
from trimesh import creation, transformations
import numpy as np
import math
import sys
from scipy.spatial.transform import Rotation as R
import csv
import random
import pandas as pd


# %%
### LOAD IN STL MODELS ####
print("Loading environment stl...")
envMesh = trimesh.load_mesh("C:/Users/mcgra/source/repos/AMP_FinalProject_Python/AMP_FinalProject_Python/FlatSlopes_WithTrees.stl",process=False)
envMesh = trimesh.Trimesh(envMesh.vertices,envMesh.faces,envMesh.face_normals,envMesh.vertex_normals)
print("Done.")
df = pd.read_csv("TrajectoryPoints.csv",sep="\n\n")
envMesh.export('grouped\envMesh.stl')
i=0
for index, row in df.iterrows():
    row = row[0].split(',')
    x = float(row[0])
    y = float(row[1])
    z = float(row[2])
    print((x,y,z))
    sphere = trimesh.creation.icosphere(radius=0.48/2,color=(0,0,255))
    sphere.apply_translation([x,y,z])
    sphere.export('grouped\sphere'+str(i)+'.stl')
    i+= 1
#    envMesh.union(sphere)
#envMesh.export('GROUP_MESH.stl')

