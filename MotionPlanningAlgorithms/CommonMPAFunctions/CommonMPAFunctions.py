import trimesh
from trimesh import viewer
from trimesh.viewer import windowed
from trimesh import creation, transformations
import numpy as np
import math
import sys
from scipy.spatial.transform import Rotation as R
import random


def Runge_Kutta_Integration_fourth_order(T_sampled, input_array, dynamic_bounds):
    None


def Get3Ddist(xyz1, xyz2):
    dist = math.sqrt(
        (xyz1[0] - xyz2[0]) ** 2 + (xyz1[1] - xyz2[1]) ** 2 + (xyz1[1] - xyz2[1]) ** 2
    )
    # print("DIST: " + str(dist))
    return dist


def CreateRotationMatrix(alpha=0, beta=0, gamma=0):
    # NO INPUTS CREATES IDENTITY MATRIX
    r_x = transformations.rotation_matrix(alpha, [1, 0, 0])
    r_y = transformations.rotation_matrix(beta, [0, 1, 0])
    r_z = transformations.rotation_matrix(gamma, [0, 0, 1])
    R = np.matmul(np.matmul(r_z, r_y), r_x)
    return R


def CombineRotTransMatrices(rotationMatrix, translationMatrix):
    rotMatShape = np.shape(rotationMatrix)
    tranMatShape = np.shape(translationMatrix)
    if rotMatShape != (4, 4) and rotMatShape != (3, 3):
        raise Exception(
            "Expecting rotation matrix with shape (4, 4) or (3, 3), given matrix of shape "
            + str(rotMatShape)
        )
        sys.exit()
    elif tranMatShape != (4, 4) and tranMatShape != (4, 1):
        raise Exception(
            "Expecting translation Matrix matrix with shape (4, 4) or (4, 1), given matrix of shape "
            + str(tranMatShape)
        )
        sys.exit()

    tranMat = translationMatrix[0:3, 3]
    combinedMatrix = np.zeros([4, 4])

    if rotMatShape != (3, 3):
        rotMat = rotationMatrix[0:3, 0:3]
        rotMatTrans = rotationMatrix[0:3, 3]
        combinedMatrix[0:3, 3] = tranMat + rotMatTrans
    else:
        rotMat = rotationMatrix
        combinedMatrix[0:3, 3] = tranMat

    combinedMatrix[0:3, 0:3] = rotMat
    combinedMatrix[3, :] = np.array([0, 0, 0, 1])
    return combinedMatrix


def CheckForDroneCollision(
    collision_manager_with_env=trimesh.collision.CollisionManager().__init__,
    drone_mesh=None,
    transform=None,
):
    collision_manager_with_env.add_object("Drone", drone_mesh, transform=transform)
    isCollision = collision_manager_with_env.in_collision_internal()
    collision_manager_with_env.remove_object("Drone")
    return isCollision


def CalculateFinalTransform(TransformationList):
    # Transformation list is a list of transformations from 1st to last
    # THIS WONT WORK -  NEED TO FIGURE OUT

    prevTransform = np.zeros([4, 4])
    for i in range(len(TransformationList), 0, -1):
        if i == 0:
            break
        else:
            if i == len(TransformationList):
                prevTransform = np.matmul(
                    TransformationList[i], TransformationList[i - 1]
                )
            else:
                prevTransform = np.matmul(prevTransform, TransformationList[i - 1])

    return prevTransform  ####XXXXXX
