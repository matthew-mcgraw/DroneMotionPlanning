#### Author: Matthew McGraw ####
#### Date:   12/03/2022     ####

import trimesh
from trimesh import viewer
from trimesh.viewer import windowed
from trimesh import creation, transformations
import numpy as np
import pyrender
import math
import sys
from scipy.spatial.transform import Rotation as R
import random

pi = math.pi

#Set up all parameters to be used in the planner here
yaw_rate_bounds = [-pi/6, pi/6]
pitch_rate_bounds = [-pi/6, pi/6]
lin_acc_rate_bounds = [-0.5, 0.5]

pitch_bounds = [-pi/3,pi/3]
velocity_bounds = [-1,1]
delta_s = 0.8
delta_bn = 2
T_prop = 2
dynamic_bounds = {"PITCH":pitch_bounds,"VEL":velocity_bounds}

N = 1000 #number of iterations
STATE_SPACE = {"X":None,"Y":None,"Z":None,"PITCH":None,"YAW":None,"VEL":None}
U = {"PITCH_RATE":yaw_rate_bounds,"YAW_RATE":pitch_rate_bounds,"LIN_ACC":lin_acc_rate_bounds}


#print("Loading environment stl...")
#envMesh = trimesh.load_mesh("C:/Users/mcgra/source/repos/AMP_FinalProject_Python/AMP_FinalProject_Python/FlatSlopes_WithTrees.stl",process=False)
#print("Done.")

#print("Loading ground mesh stl...")
#groundMesh = trimesh.load_mesh("C:/Users/mcgra/source/repos/AMP_FinalProject_Python/AMP_FinalProject_Python/FlatSlopes_NoTrees.stl")
#groundMesh = trimesh.Trimesh(groundMesh.vertices,groundMesh.faces,groundMesh.face_normals,groundMesh.vertex_normals)
#groundMeshZTransform = transformations.translation_matrix([0,0,15])
#groundMesh2 = groundMesh.copy().apply_transform(groundMeshZTransform)
#boundingMesh = trimesh.load_mesh("C:/Users/mcgra/source/repos/AMP_FinalProject_Python/AMP_FinalProject_Python/FlatSlopes_BoundingVolume.stl")
#boundingMesh = trimesh.Trimesh(boundingMesh.vertices,boundingMesh.faces,boundingMesh.face_normals,boundingMesh.vertex_normals)

#x,y,z = trimesh.sample.volume_mesh(boundingMesh,1)[0]
#print(x)
#print(y)
#print(z)
#x,y,z = trimesh.sample.volume_mesh(boundingMesh,1)[0]
#print(x)
#print(y)
#print(z)
#x,y,z = trimesh.sample.volume_mesh(boundingMesh,1)[0]
#print(x)
#print(y)
#print(z)
#x,y,z = trimesh.sample.volume_mesh(boundingMesh,1)[0]
#print(x)
#print(y)
#input(z)

#boundaryMesh = trimesh.util.concatenate(groundMesh,groundMesh2)
#trimesh.repair.fill_holes(boundaryMesh)
#groundScene = trimesh.Scene()
#groundScene.add_geometry(boundaryMesh)
#groundScene.show()
##groundMesh.
##groundMesh = trimesh.load_mesh("C:/Users/mcgra/source/repos/AMP_FinalProject_Python/AMP_FinalProject_Python/FlatSlopes_NoTrees.stl")
#print("Done.")

##need to take ground mesh, offset it in positive z, and concatenate and seal meshes
##groundMesh.

#print("Loading drone stl...")
#droneMesh = trimesh.load_mesh("C:/Users/mcgra/source/repos/AMP_FinalProject_Python/AMP_FinalProject_Python/DroneGeneral.obj")
#print("Done.")

#trans_matrix = transformations.translation_matrix([2,2,2])

#***** IN ORDER TO PROPERLY APPLY TRANSFORM, MUST ROTATE, THEN TRANSLATE. ORRRRRR REZERO AFTER TRANSLATION


def Runge_Kutta_Integration_fourth_order(T_sampled,input_array,dynamic_bounds):
    None



def CreateRotationMatrix(alpha = 0, beta = 0, gamma = 0):
    #NO INPUTS CREATES IDENTITY MATRIX
    r_x = transformations.rotation_matrix(alpha,[1,0,0])
    r_y = transformations.rotation_matrix(beta,[0,1,0])
    r_z = transformations.rotation_matrix(gamma,[0,0,1])
    R = np.matmul(np.matmul(r_z,r_y),r_x)
    return R


def CombineRotTransMatrices(rotationMatrix,translationMatrix):
    rotMatShape = np.shape(rotationMatrix)
    tranMatShape = np.shape(translationMatrix)
    if(rotMatShape != (4,4) and rotMatShape != (3,3)):
        raise Exception("Expecting rotation matrix with shape (4, 4) or (3, 3), given matrix of shape "+str(rotMatShape))
        sys.exit()
    elif(tranMatShape != (4,4) and tranMatShape != (4,1)):
        raise Exception("Expecting translation Matrix matrix with shape (4, 4) or (4, 1), given matrix of shape "+str(tranMatShape))
        sys.exit()

        
    tranMat = translationMatrix[0:3,3]
    combinedMatrix = np.zeros([4,4])

    if(rotMatShape != (3,3)):
        rotMat = rotationMatrix[0:3,0:3]
        rotMatTrans = rotationMatrix[0:3,3]
        combinedMatrix[0:3,3] = tranMat + rotMatTrans
    else:
        rotMat = rotationMatrix
        combinedMatrix[0:3,3] = tranMat

    combinedMatrix[0:3,0:3] = rotMat
    combinedMatrix[3,:] = np.array([0,0,0,1])
    return combinedMatrix

def CheckForDroneCollision(collision_manager_with_env = trimesh.collision.CollisionManager().__init__,drone_mesh=None,transform=None):
    collision_manager_with_env.add_object("Drone",drone_mesh,transform=transform)
    isCollision = collision_manager_with_env.in_collision_internal()
    collision_manager_with_env.remove_object("Drone")
    return isCollision

def CalculateFinalTransform(TransformationList):
    #Transformation list is a list of transformations from 1st to last
    #THIS WONT WORK -  NEED TO FIGURE OUT

    prevTransform = np.zeros([4,4])
    for i in range(len(TransformationList),0,-1):
        if(i == 0):
            break
        else:
            if(i == len(TransformationList)):
                prevTransform = np.matmul(TransformationList[i],TransformationList[i-1])
            else:
                prevTransform = np.matmul(prevTransform,TransformationList[i-1])

    return prevTransform ####XXXXXX

class MeshObject:
    def __init__(self,mesh,name):
        self.name = name
        self.mesh = mesh

        self.scene = trimesh.Scene()
        self.scene.add_geometry(mesh)
        self.centroid = self.scene.centroid
        self.X = self.centroid[0]
        self.Y = self.centroid[1]
        self.Z = self.centroid[2]
        self.alpha = 0 #rotation about X
        self.beta = 0  #rotation about Y
        self.gamma = 0 #rotation about Z
        self.ROTATION_MATRIX = np.array([[1,0,0],[0,1,0],[0,0,1]])
        self.TRANSLATION_MATRIX = np.array([self.X,self.Y,self.Z])
    
    def GetEulerAnglesFromMatrix(self):
        r = R.from_matrix(self.ROTATION_MATRIX)
        xyz = r.as_euler('xyz',degrees=False)
        self.alpha = xyz[0]
        self.beta = xyz[1]
        self.gamma = xyz[2]

    def RotateMesh(self,rot_mat):
        self.ROTATION_MATRIX = np.matmul(self.ROTATION_MATRIX,rot_mat)
        self.GetEulerAnglesFromMatrix()

    def TranslateMesh(self,transl_mat):
        self.TRANSLATION_MATRIX = np.add(TRANSLATION_MATRIX,transl_mat)
        self.X = self.TRANSLATION_MATRIX[0]
        self.Y = self.TRANSLATION_MATRIX[1]
        self.Z = self.TRANSLATION_MATRIX[2]
        
    def GetMeshCentroid(self,mesh):
        scene = trimesh.Scene()
        scene.add_geometry(self.mesh)
        centroid = scene.centroid
        return centroid

    def CalculateTrajectory():
        None


class CollisionChecker:
    def __init__(self,envMesh,botMesh):
        self.envMesh = envMesh

class StateValidation:
    def __init__(self):
        None

    #State sample will return a sample state
    def StateSample(boundingMesh=None,x_bounds=None,y_bounds=None,z_bounds=None,alpha_bounds=None,beta_bounds=None,gamma_bounds=None,velocity_bounds=None):
        if(boundingMesh is not None):
            x,y,z = trimesh.sample.volume_mesh(boundingMesh,1)[0]
            alpha = random.uniform(alpha_bounds[0],alpha_bounds[1])
            beta = random.uniform(beta_bounds[0],beta_bounds[1])
            gamma = random.uniform(gamma_bounds[0],gamma_bounds[1])
            velocity = random.uniform(velocity_bounds[0],velocity_bounds[1])

        else:
            x = random.uniform(x_bounds[0], x_bounds[1])
            y = random.uniform(y_bounds[0], y_bounds[1])
            z = random.uniform(z_bounds[0], z_bounds[1])
            alpha = random.uniform(alpha_bounds[0],alpha_bounds[1])
            beta = random.uniform(beta_bounds[0],beta_bounds[1])
            gamma = random.uniform(gamma_bounds[0],gamma_bounds[1])
            velocity = random.uniform(velocity_bounds[0],velocity_bounds[1])
    
class PlanningAlgorithms:
    def __init__(self):
        None

    class SST: #We will be using the drone model from the exam
        def __init__(self,start_state=None,goal_state=None,u_bounds_list=None,StateSpace=None):
            self.start_state = start_state
            self.goal_state = goal_state
            self.Witness_Regions = WitnessRegions()
            self.Active_Set = ActiveSet()
            self.Inactive_Set = InactiveSet()
            #self.Active_Set.Has_Witness = False
            self.StateSpace = StateSpace
            None
        def SST(X,U,x_0,T_prop,N,delta_BestNeighbor,delta_s):
            None

        def Is_Node_Locally_Best_SST(X_new,S,delta_s):
            None
        def Prune_Dominated_Nodes_SST(X_new,V_active,V_inactive,E):
            None
        def Best_First_Selection_SST(X,V,delta_BestNeighbor):
            x_rand = StateValidation.StateSample(boundingMesh,alpha_bounds=alpha_bounds)
            None
        def MonteCarlo_Prop(x_selected,U,T_prop):
            #x_selected is chosen neighbor, U is input bounds dict, T is max propagation time
            numInputs = len(U)
            random_inputs = np.empty([1,len(u_bounds_list)])
            for input in range(0,numInputs):
                inputBounds = input.value #get bounds of input from dictionary
                inputSample = random.uniform(inputBounds[0],inputBounds[1])
                random_inputs[0][i] = random.uniform(u_bounds_list[i][0],u_bounds_list[i][1])
            t = random.uniform(0,T_Prop)




        def Randomized_Inputs(u_bounds_list):
            random_inputs = np.empty([1,len(u_bounds_list)])
            numInputs = len(u_bounds_list)
            
            for i in range(0,numInputs):
                random_inputs[0][i] = random.uniform(u_bounds_list[i][0],u_bounds_list[i][1])
            return random_inputs

        class ActiveSet:
            def __init__(self):
                self.States = []
                self.States.append(self.start_state)
                None
        class InactiveSet:
            def __init__(self):
                None
        class WitnessRegions:
            def __init__(self):
                None

            def CreateNewRegion():
                #Use trimesh to create spherical region
                None

        

input(PlanningAlgorithms.SST.Randomized_Inputs([yaw_rate_bounds,pitch_rate_bounds,lin_acc_rate_bounds]))


droneMeshOBJ = MeshObject(droneMesh,"DRONE")
envMeshOBJ = MeshObject(envMesh,"ENV")
groundMeshOBJ = MeshObject(groundMesh,"GROUND")

#CREATE THE COLLISION MANAGER WITH OUR ENVIRONMENT
ENV_COLIS_MNGR = trimesh.collision.CollisionManager()
ENV_COLIS_MNGR.add_object("ENV",envMesh)


CreateRotationMatrix(alpha = math.pi/2)
angle = math.pi/2

#rotMatrix = CreateRotationMatrix(alpha=alpha)

rotMatrix = CreateRotationMatrix(angle,angle,0)
translationMatrix = transformations.translation_matrix([300,-15,10])

transformationMatrix = CombineRotTransMatrices(rotMatrix,translationMatrix)


droneScene = trimesh.Scene()
droneScene.add_geometry(droneMesh,node_name="drn",geom_name="drn")
centroid = droneScene.centroid 
droneScene.add_geometry(envMesh)
droneScene.set_camera(center=[0,0,0])
print(centroid)
droneScene.add_geometry(boundaryMesh)
droneScene.show()

#CHECKING FOR COLLISIONS
COLIS_MNGR = trimesh.collision.CollisionManager()
print("Adding env to its collision manager...")
COLIS_MNGR.add_object("Env",envMesh)
print("Done.")

#print("Adding drone to its collision manager...")
#COLIS_MNGR.add_object("Drone",droneMesh,transform=transformationMatrix)
#print("Done.")

collision = CheckForDroneCollision(COLIS_MNGR,droneMesh,transformationMatrix)
print("Position 1 collision?")
print(collision)
droneScene.delete_geometry("drn")

translationMatrix2 = transformations.translation_matrix([-152,-135,0])
rotMatrix2 = CreateRotationMatrix()
transformMat2 = CombineRotTransMatrices(rotMatrix2,translationMatrix2)
finalTransform = np.matmul(transformMat2, transformationMatrix)
droneScene.add_geometry(droneMesh,node_name="drn",geom_name="drn",transform=finalTransform)
collision = CheckForDroneCollision(COLIS_MNGR,droneMesh,finalTransform)
print("Position 2 collision?")
print(collision)
droneScene.show()


