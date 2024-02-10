import numpy as np
from geometry_msgs.msg import Vector3

class vectorCalculator:

    @staticmethod
    def vector3_to_numpy(vector3):
        return np.array([vector3.x, vector3.y, vector3.z])
    
    @staticmethod
    def numpy_to_vector3(np_array):
        return Vector3(x=np_array[0], y=np_array[1], z=np_array[2])
    
    @staticmethod
    def add_two_vectors(vector1, vector2):
        nVector1 = vectorCalculator.vector3_to_numpy(vector1)
        nVector2 = vectorCalculator.vector3_to_numpy(vector2)
        nSum = nVector1+nVector2
        rSum = vectorCalculator.numpy_to_vector3(nSum)
        return rSum
    
    @staticmethod
    def subtract_two_vectors(vector1, vector2):
        nVector1 = vectorCalculator.vector3_to_numpy(vector1)
        nVector2 = vectorCalculator.vector3_to_numpy(vector2)
        nSum = nVector1-nVector2
        rSum = vectorCalculator.numpy_to_vector3(nSum)
        return rSum
    
    