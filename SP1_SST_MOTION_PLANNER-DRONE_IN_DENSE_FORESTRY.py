# %%
import trimesh
from trimesh import viewer
from trimesh.viewer import windowed
from trimesh import creation, transformations
from scipy.integrate import odeint
import numpy as np
import math
import sys
from scipy.spatial.transform import Rotation as R
import random
import csv

# %%
#Set up all parameters to be used in the planner here
pi = math.pi
yaw_rate_bounds = [-pi/12, pi/12]
pitch_rate_bounds = [-pi/12, pi/12]
lin_acc_rate_bounds = [-0.35, 0.35]

pitch_bounds = [-pi/3,pi/3]
velocity_bounds = [-1,1]
delta_s = 1.5
delta_bn = 2
T_prop = 8
probability_goal = 0.05
dynamic_bounds = {"PITCH":pitch_bounds,"VEL":velocity_bounds}
goal_state = [15, -300, 5, 0,0,0]
goal_radius = 2

N = 10000 #number of iterations
STATE_SPACE = {"X":None,"Y":None,"Z":None,"PITCH":None,"YAW":None,"VEL":None}
# U = {"PITCH_RATE":yaw_rate_bounds,"YAW_RATE":pitch_rate_bounds,"LIN_ACC":lin_acc_rate_bounds}
U = [yaw_rate_bounds,pitch_rate_bounds,lin_acc_rate_bounds]

# %%
#### LOAD IN STL MODELS ####
#########print("Loading environment stl...")
envMesh = trimesh.load_mesh("C:/Users/mcgra/source/repos/AMP_FinalProject_Python/AMP_FinalProject_Python/FlatSlopes_WithTrees.stl",process=False)
envMesh = trimesh.Trimesh(envMesh.vertices,envMesh.faces,envMesh.face_normals,envMesh.vertex_normals)
#####print("Done.")


# %%
print("Loading drone stl...")
droneSTL = trimesh.load_mesh("C:/Users/mcgra/source/repos/AMP_FinalProject_Python/AMP_FinalProject_Python/DroneGeneral_Smaller.stl")
droneSTL = trimesh.Trimesh(droneSTL.vertices,droneSTL.faces,droneSTL.face_normals,droneSTL.vertex_normals)
print("Done.")
#Get max dimension of the drone
bounds = droneSTL.bounds
x_dim = bounds[1][0]-bounds[0][0]
y_dim = bounds[1][1]-bounds[0][1]
z_dim = bounds[1][2]-bounds[0][2]
droneMaxDim = math.sqrt(x_dim**2+y_dim**2+z_dim**2)
print("Drone Max Dimension: " + str(droneMaxDim))

# %%

print("Loading bounding mesh stl...")
boundingMesh = trimesh.load_mesh("C:/Users/mcgra/source/repos/AMP_FinalProject_Python/AMP_FinalProject_Python/FlatSlopes_BoundingVolume.stl")
boundingMesh = trimesh.Trimesh(boundingMesh.vertices,boundingMesh.faces,boundingMesh.face_normals,boundingMesh.vertex_normals)
print("Done.")


# %%

print("Loading drone obj...")
droneMesh = trimesh.load_mesh("C:/Users/mcgra/source/repos/AMP_FinalProject_Python/AMP_FinalProject_Python/DroneGeneral_Smaller.obj")
#droneMesh = trimesh.Trimesh(droneMesh.vertices,droneMesh.faces,droneMesh.face_normals,droneMesh.vertex_normals)
print("Done.")


# %%
print("Loading drone stl...")
droneSTL = trimesh.load_mesh("C:/Users/mcgra/source/repos/AMP_FinalProject_Python/AMP_FinalProject_Python/DroneGeneral_Smaller.stl")
droneSTL = trimesh.Trimesh(droneSTL.vertices,droneSTL.faces,droneSTL.face_normals,droneSTL.vertex_normals)
print("Done.")
#Get max dimension of the drone
bounds = droneSTL.bounds
x_dim = bounds[1][0]-bounds[0][0]
y_dim = bounds[1][1]-bounds[0][1]
z_dim = bounds[1][2]-bounds[0][2]
droneMaxDim = math.sqrt(x_dim**2+y_dim**2+z_dim**2)
print("Drone Max Dimension: " + str(droneMaxDim))

# %%
def Runge_Kutta_Integration_fourth_order(T_sampled,input_array,dynamic_bounds):
    None

def Get3Ddist(xyz1,xyz2):
    dist = math.sqrt((xyz1[0]-xyz2[0])**2 + (xyz1[1]-xyz2[1])**2 + (xyz1[1]-xyz2[1])**2)
    # print("DIST: " + str(dist))
    return dist


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

# %%
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
        self.alpha = 0 #rotation about X, roll
        self.beta = 0  #rotation about Y, pitch
        self.gamma = 0 #rotation about Z, yaw
        self.velocity = 0
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
        self.TRANSLATION_MATRIX = np.add(self.TRANSLATION_MATRIX,transl_mat)
        self.X = self.TRANSLATION_MATRIX[0]
        self.Y = self.TRANSLATION_MATRIX[1]
        self.Z = self.TRANSLATION_MATRIX[2]
        
    def GetMeshCentroid(self,mesh):
        scene = trimesh.Scene()
        scene.add_geometry(self.mesh)
        centroid = scene.centroid
        return centroid


# %%
class Vertice:
    def __init__(self,state,cost):
        self.state=state
        self.cost = cost
        self.Parent = None
        self.Children = []

    def AddParent(self,parent):
        self.Parent = parent
    def AddChild(self,child):
        self.Children.append(child)


# %%
class PlanningAlgorithms:
    def __init__(self):
        None

    class SST: #We will be using the drone model from the exam
        def __init__(self,start_state=None,goal_state=None,u_bounds_list=None,StateSpace=None):
            self.start_state = start_state
            self.goal_state = goal_state
            self.Witness_Regions = self.WitnessRegions()
            self.Active_Set = self.ActiveSet()
            self.Inactive_Set = self.InactiveSet()
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
            x_rand = StateValidation.StateSample(boundingMesh,pitch_bounds=pitch_bounds,velocity_bounds=velocity_bounds)
            x_nearList = [(Get3Ddist(V[i][0:3],x_rand[0:3]),i) for i in range(0,len(V)) if Get3Ddist(V[i][0:3],x_rand[0:3]) < delta_BestNeighbor]
            x_near = [V[i] for i in range(0,len(V)) if Get3Ddist(V[i][0:3],x_rand[0:3]) < delta_BestNeighbor]
            if(len(x_near) == 0): #no neighbors within delta_BestNeighbor
                None

        # def MonteCarlo_Prop(x_selected,U,T_prop):
        #     #x_selected is chosen neighbor, U is input bounds dict, T is max propagation time
        #     numInputs = len(U)
        #     random_inputs = np.empty([1,numInputs])
        #     for input_ in range(0,numInputs):
        #         inputBounds = input.value #get bounds of input from dictionary
        #         inputSample = random.uniform(inputBounds[0],inputBounds[1])
        #         random_inputs[0][input] = random.uniform(U[input][0],U[input][1])
        #     t = random.uniform(0,T_prop)

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
            def __init__(self,x_0,d_s,cost):
                self.Regions = []
                self.regionID = 0
                region = self.Region(x_0,d_s,cost,self.regionID)
                self.Regions.append(region)
                
            def AddNewRegion(self,rep,d_s,cost,regionID):
                region = self.Region(rep,d_s,cost,regionID)
                self.Regions.append(region)
                self.regionID = self.regionID + 1

            class Region:
                def __init__(self,rep,d_s,cost,regionID):
                    self.representative = rep #current state that represents the region
                    x = rep.state[0]
                    y = rep.state[1]
                    z = rep.state[2]
                    self.cost = cost
                    regionSphere = trimesh.creation.icosphere(3,d_s)
                    regionSphere.apply_translation([x,y,z])
                    self.regionMesh = regionSphere
                    self.regionID = regionID
                    
                def ChangeRepresentative(self,x,cost):
                    self.representative = x
                    self.cost = cost

# %%
droneMeshOBJ = MeshObject(droneMesh,"DRONE")
x_0 = [droneMeshOBJ.X,droneMeshOBJ.Y,droneMeshOBJ.Z,droneMeshOBJ.beta,droneMeshOBJ.gamma,droneMeshOBJ.velocity]
envMeshOBJ = MeshObject(envMesh,"ENV")
boundaryMeshOBJ = MeshObject(boundingMesh,"BOUNDARY")


# %%
class Vertice:
    def __init__(self,state,cost):
        self.state=state
        self.cost = cost
        self.Parent = None
        self.Children = []

    def AddParent(self,parent):
        self.Parent = parent
    def AddChild(self,child):
        self.Children.append(child)


# %%
#CREATE THE COLLISION MANAGER WITH OUR ENVIRONMENT
print("Creating collision manager with environment...")
ENV_COLIS_MNGR = trimesh.collision.CollisionManager()
ENV_COLIS_MNGR.add_object("ENV",envMesh)
print("Done.")

# %%
class StateSampler:
    def __init__(self,boundingMesh):
        self.boundingMesh = boundingMesh

    def StateSample(self,x_bounds=None,y_bounds=None,z_bounds=None,pitch_bounds=None,beta_bounds=None,yaw_bounds=None,velocity_bounds=None):
            if(self.boundingMesh is not None):
                boundingMesh = self.boundingMesh
                sample = None
                while(sample is None or len(sample) == 0):
                    sample = trimesh.sample.volume_mesh(boundingMesh,1)
                x = sample[0][0]
                y = sample[0][1]
                z = sample[0][2]
                #alpha = random.uniform(alpha_bounds[0],alpha_bounds[1])
                pitch = random.uniform(pitch_bounds[0],pitch_bounds[1])
                yaw = random.uniform(0,2*pi)
                velocity = random.uniform(velocity_bounds[0],velocity_bounds[1])

            else:
                x = random.uniform(x_bounds[0], x_bounds[1])
                y = random.uniform(y_bounds[0], y_bounds[1])
                z = random.uniform(z_bounds[0], z_bounds[1])
                #alpha = random.uniform(alpha_bounds[0],alpha_bounds[1])
                pitch = random.uniform(pitch_bounds[0],pitch_bounds[1])
                yaw = random.uniform(0,2*pi)
                velocity = random.uniform(velocity_bounds[0],velocity_bounds[1])

            return [x,y,z,pitch,yaw,velocity]

# %%
def ExtendBranch(state0,state1,delta_bn):
    #Get unit vector <x,y,z>
    state_diff = np.subtract(state1,state0)
    # print("STATE 0: " +str(state0))
    # print(state_diff)
    vector = state_diff[0:3]
    abs_vector = math.sqrt(vector[0]**2+vector[1]**2+vector[2]**2)
    unit_vector = vector/abs_vector
    extension = unit_vector*delta_bn
    
    new_xyz = state0[0:3]+extension
    x_new = [new_xyz[0],new_xyz[1],new_xyz[2],state1[3],state1[4],state1[5]]
    return x_new

# %%
def Best_First_Selection_SST(X,V,delta_BestNeighbor,sampler):
            useGoal = random.uniform(0,1)
            if(useGoal > 0.95):
                x_rand = goal_state
            else:
                x_rand = sampler.StateSample(sampler.boundingMesh,pitch_bounds=pitch_bounds,velocity_bounds=velocity_bounds)
            # print(x_rand)
            # x_nearList = [(Get3Ddist(V[i].state[0:3],x_rand[0:3]),i) for i in range(0,len(V)) if Get3Ddist(V[i].state[0:3],x_rand[0:3]) < delta_BestNeighbor]
            X_near_within_delBN = [[V[i].cost,V[i]] for i in range(0,len(V)) if Get3Ddist(V[i].state[0:3],x_rand[0:3]) < delta_BestNeighbor]
            X_near_distances = [[V[i],Get3Ddist(V[i].state[0:3],x_rand[0:3])] for i in range(0,len(V))]
            X_near_distances = sorted(X_near_distances, key = lambda x: x[1])
            # print("X_NEARLIST: "+str(x_near))
            
            closest = X_near_distances[0][0]
            # print("CLOSEST NEIGHBOR: " x)
            # print(closest)
            if(len(X_near_within_delBN) == 0): #no neighbors within delta_BestNeighbor, use closest neighbor and extend branch by delta_BN
                x_selected = closest
                # x_new = ExtendBranch(V[closest].state,x_rand,delta_BestNeighbor)
                # print("X_NEW: "+str(x_new))
                mode = "Branch Extension"
                return x_selected, mode
            else:
                X_near_within_delBN.sort(key=lambda x: x[0])
                x_selected = X_near_within_delBN[0][1]
                # # print("X NEAR LIST: " +str(x_nearList))
                # x_new_parent_idx = x_nearList[0][1]
                mode = "Best Neighbor"
                return x_selected, mode

# %%
def Is_Node_Locally_the_Best_SST(xnew, S = PlanningAlgorithms.SST.WitnessRegions.__init__, delta_s=1):
    rgnIdx = 0
    regionID = None
    inOldRegion = False
    for witness_region in S.Regions: #check if new or old cost is better and make that one region rep
        regionID = witness_region.regionID
        if(witness_region.regionMesh.contains(([xnew.state[0:3]]))):
            if(witness_region.cost > xnew.cost):
                # witness_region.ChangeRepresentative(xnew,xnew.cost)
                regionID = witness_region.regionID
                inOldRegion = True
                return True, regionID, inOldRegion
            else:
                return False, regionID, inOldRegion
        rgnIdx += 1
            

    #if it reaches this point, it was not in an old region, new witness region will be created
    regionID += 1
    S.AddNewRegion(xnew,delta_s,xnew.cost,regionID)
    rgnIdx += 1
    inOldRegion = False
    return True, regionID, inOldRegion
        

# %%
def Prune_Dominated_Nodes_SST(xnew, V_active, V_inactive, E, rgnIdx, S,isOldRegion):
    s_new = None
    idx = 0
    for i in range(0,len(S.Regions)):
        if(S.Regions[i].regionID == rgnID):
            s_new = S.Regions[i]
            break
        else:
            idx+=1
    # s_new = [S.Regions[i] for i in range(0,len(S.Regions)) if S.Regions[i].regionID == rgnID] #S.Regions[rgnIdx] #witness region on new node
    x_peer = s_new.representative
    if(isOldRegion and x_peer is not None): #move x_peer from active to inactive
        V_active.remove(x_peer)             
        V_inactive.append(x_peer)   
    S.Regions[idx].representative = xnew
    while(x_peer is not None and len(x_peer.Children)==0 and x_peer in V_inactive):
        print(x_peer)
        x_parent = x_peer.Parent
        print(x_parent)
        if([x_parent,x_peer] in E):
            E.remove([x_parent,x_peer])
        V_inactive.remove(x_peer)
        x_peer = x_parent

    

# %%
def CalculateTrajectory(state,inputs,t):
    pi = math.pi
    cos = math.cos
    sin = math.sin
    x_0 = (state[0], state[1], state[2], state[3], state[4], state[5]) #x:300, y:-15, z:10, theta:0, yaw: 0, v:0
    #TRIMESH SWEEP DOESN"T APPEAR TO BE WORKING, JUST USE ICOSPHERES INSTEAD
    #x_0 is the state at the x_selected
    omega = inputs[0]
    alpha = inputs[1]
    acc = inputs[2]
    # print("X_0 VALUES: "+str(x_0))
    # print(omega)
    # print(alpha)
    # print(acc)

    def dXdt(X,t):
        x,y,z,yaw,theta,v = X
        return [v*cos(yaw)*cos(theta),
                v*sin(yaw)*cos(theta),
                v*sin(theta),
                omega,
                alpha,
                acc]

    # t_inc = None
    # if(int(t) == 0):
    #     t_inc = 1
    # else:
    #     t_inc = int(t*10)
    t = np.linspace(0,t,10)
    sol = odeint(dXdt,x_0,t)
    x,y,z,yaw,theta,v = sol.T
    return x,y,z,yaw,theta,v
    # print((x[-1],y[-1],z[-1],yaw[-1],theta[-1],v[-1]))

    # ax = plt.axes(projection='3d')
    # ax.plot3D(x,y,z)
    # plt.show()

# %%
def IsTrajectoryInCollision(collision_manager_with_env,botRadius, xyzPoints):
    isCollision = False
    # xyzPoints = list(xyzPath)
    for [x,y,z] in xyzPoints:
        sphere = trimesh.creation.icosphere(radius=botRadius)
        sphere = sphere.apply_translation([x,y,z])

        collision_manager_with_env.add_object("Drone",sphere)
        isCollision = collision_manager_with_env.in_collision_internal()
        collision_manager_with_env.remove_object("Drone")
        if(isCollision == True):
            break
    return isCollision

# %%
def MonteCarlo_Prop(x_selected,U,T_prop):
    #x_selected is chosen neighbor, U is input bounds dict, T is max propagation time
    numInputs = len(U)
    random_inputs = np.empty([1,numInputs])
    i = 0
    for input_bounds in U:
        # inputBounds = input.value #get bounds of input from dictionary
        # inputSample = random.uniform(inputBounds[0],inputBounds[1])
        random_inputs[0][i] = random.uniform(input_bounds[0],input_bounds[1])
        i+= 1
    t = random.uniform(0,T_prop)
    return random_inputs, t
        

# %%
def In_Goal_Region(state,goal_state=goal_state,goal_radius=goal_radius):
    Goal_Sphere = trimesh.creation.icosphere(radius=goal_radius)
    Goal_Sphere = Goal_Sphere.apply_translation(goal_state[0:3])
    if(Goal_Sphere.contains([state[0:3]])):
        return True
    else:
        return False


# %%
def CalculateCost(xyzPoints):
    cost = 0
    for i in range(0,len(xyzPoints)-1):
        cost = cost+Get3Ddist(xyzPoints[i],xyzPoints[i+1])
    return cost

# %%
E = [] #trajectory list
V_active = [] #active state list
initialState = Vertice(x_0,0)
V_active.append(initialState)
V_inactive = [] #inactive state list
V = V_active+V_inactive #active and inactive states
G = [V,E]
S = PlanningAlgorithms.SST.WitnessRegions(initialState,delta_s,0)
print(V[0].state[0:3])

# x,y,z = trimesh.sample.volume_mesh(boundingMesh,1)[0]
# print(x)
# print(y)
# print(z)

ss_sampler = StateSampler(boundingMesh)
x_selected,  mode = Best_First_Selection_SST(STATE_SPACE,V_active,delta_bn,ss_sampler)
# print(x_selected)
# print(x_new_parentIdx)
# print(ss_sampler.StateSample(pitch_bounds=pitch_bounds,velocity_bounds=velocity_bounds))
scene = trimesh.Scene()
scene.add_geometry(envMesh)
for iteration in range(0,N):
    print(iteration)
    print("ACTIVE NODES: " + str(len(V_active)))
    print("INACTIVE NODES: " + str(len(V_inactive)))
    print("WITNESS REGIONS: " + str(len(S.Regions)))
    x_selected, mode = Best_First_Selection_SST(STATE_SPACE,V_active,delta_bn,ss_sampler)
    # print("PARENT INDEX: "+str(x_new_parentIdx)+", MODE: "+mode)
    random_inputs, t = MonteCarlo_Prop(x_selected,U,T_prop)
    random_inputs = random_inputs[0]

    # print(x_selected)
    # print(random_inputs)
    # print(t)
    x,y,z,phi,theta,v = CalculateTrajectory(x_selected.state,random_inputs,t)
    xyzPoints = []
    allPointsInBounds = True
    for i in range(0,len(x)):
        if(not boundingMesh.contains([[x[i],y[i],z[i]]])):
            allPointsInBounds = False
            break
        if(theta[i] < pitch_bounds[0] or theta[i] > pitch_bounds[1]):
            allPointsInBounds = False
            break
        if(v[i] < velocity_bounds[0] or v[i] > velocity_bounds[1]):
            allPointsInBounds = False
            break

        xyzPoints.append([x[i],y[i],z[i]])
    if(allPointsInBounds == False):
        continue

    # xyzPath = zip(x,y,z)
    # print(xyzPoints)
    # print([x,y,z])
    # def __init__(self,state,cost):

    new_V_state = [x[-1],y[-1],z[-1],phi[-1],theta[-1],v[-1]]
    new_V_cost = CalculateCost(xyzPoints) + x_selected.cost
    x_new = Vertice(new_V_state,new_V_cost)
    x_new.AddParent(x_selected)
    parent_idx = V.index(x_selected)
    # print(new_V_state)
    

    droneTrajCollision = IsTrajectoryInCollision(ENV_COLIS_MNGR,droneMaxDim/2,xyzPoints)
    # print("DRONE COLLISION?: "+str(droneTrajCollision))
    #scene = trimesh.Scene()
    # scene.add_geometry(trajCylinder)
    # scene.add_geometry(droneSphere)
    #scene.add_geometry(envMesh)
    # scene.show()

    # if(dronePosCollision==True):
    #     print("POS COLLISION")
    # if(droneTrajCollision==True):
    #     print("TRAJ COLLISION")

    #IF NO COLLISION OCCURS, WE CAN MOVE ONTO CHECKING IF VERTICE IS BEST IN WITNESS REGION
    if(droneTrajCollision == False):
        isNodeBest, rgnID, inOldRegion = Is_Node_Locally_the_Best_SST(x_new,S=S,delta_s=delta_s)
        # print("IS NODE BEST?: " +str(isNodeBest))
        # print("IN OLD REGION?: " +str(inOldRegion))

        if(isNodeBest):
            # x_parent = 
            # V_active.remove(V_active[int(x_new_parentIdx)])
            V_active[int(parent_idx)].Children.append(x_new)
            # print(list(xyzPath))
            # V_active.append()
            V_active.append(x_new)
            # xyzPoints = list(xyzPath)
            # print(xyzPath)
            #sphereUnion=trimesh.Trimesh()
            for i in range(0,len(xyzPoints)):
                sphere = trimesh.creation.icosphere(radius=droneMaxDim/2,color=(0,0,255))
                sphere.apply_translation(xyzPoints[i])
                scene.add_geometry(sphere)
                #print("SPHERE ADDED")
            # for [x,y,z] in list(xyzPath):
            #scene.add_geometry(sphereUnion)
            
            f = open("C:/Users/mcgra/source/repos/AMP_FinalProject_Python/AMP_FinalProject_Python/TrajectoryPoints.csv",'a')
            writer = csv.writer(f)
            trajectory = [[(x_selected,x_new),(x,y,z,phi,theta,v)]]
            for i in range(0,len(x)):
                row = (x[i],y[i],z[i])
                print(row)
                writer.writerow(row)
            f.close

            E.append(trajectory)
            Prune_Dominated_Nodes_SST(x_new,V_active, V_inactive, E, rgnID, S,inOldRegion)
            V = V_active+V_inactive
            G = (V,E)
            # scene.add_geometry(droneSphere)
            # scene.add_geometry(trajCylinder)
            if(In_Goal_Region(x_new.state)):
                print("GOAL REACHED")
                break

        # print("NO COLLISION")
# for vertice in V_active:
#     print(vertice.state)


