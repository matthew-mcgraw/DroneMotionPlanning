import numpy as np
import trimesh


class MeshObject:
    def __init__(self, mesh, name):
        self.name = name
        self.mesh = mesh

        self.scene = trimesh.Scene()
        self.scene.add_geometry(mesh)
        self.centroid = self.scene.centroid
        self.X = self.centroid[0]
        self.Y = self.centroid[1]
        self.Z = self.centroid[2]
        self.alpha = 0  # rotation about X, roll
        self.beta = 0  # rotation about Y, pitch
        self.gamma = 0  # rotation about Z, yaw
        self.velocity = 0
        self.ROTATION_MATRIX = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
        self.TRANSLATION_MATRIX = np.array([self.X, self.Y, self.Z])

    def GetEulerAnglesFromMatrix(self):
        r = R.from_matrix(self.ROTATION_MATRIX)
        xyz = r.as_euler("xyz", degrees=False)
        self.alpha = xyz[0]
        self.beta = xyz[1]
        self.gamma = xyz[2]

    def RotateMesh(self, rot_mat):
        self.ROTATION_MATRIX = np.matmul(self.ROTATION_MATRIX, rot_mat)
        self.GetEulerAnglesFromMatrix()

    def TranslateMesh(self, transl_mat):
        self.TRANSLATION_MATRIX = np.add(self.TRANSLATION_MATRIX, transl_mat)
        self.X = self.TRANSLATION_MATRIX[0]
        self.Y = self.TRANSLATION_MATRIX[1]
        self.Z = self.TRANSLATION_MATRIX[2]

    def GetMeshCentroid(self, mesh):
        scene = trimesh.Scene()
        scene.add_geometry(self.mesh)
        centroid = scene.centroid
        return centroid

    def CalculateTrajectory():
        None
