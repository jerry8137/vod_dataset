import numpy as np
import math
from scipy.spatial.transform import Rotation as R

def isRotationMatrix(R) :
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6

def rotationMatrixToEulerAngles(R) :

    assert(isRotationMatrix(R))
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])

    singular = sy < 1e-6

    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0

    return np.array([x, y, z])

# a = np.array([[-0.007980200000000000, -0.999854100000000000, 0.015104900000000000, 0.151000000000000000], \
#                 [0.118497000000000000, -0.015944500000000000, -0.992826400000000000, -0.461000000000000000], \
#                 [0.992922400000000000, -0.006133100000000000, 0.118606900000000000, -0.915000000000000000]])

a = np.array([[-0.013857, -0.9997468, 0.01772762, 0.05283124], \
        [0.10934269, -0.01913807, -0.99381983, 0.98100483], \
        [0.99390751, -0.01183297, 0.1095802, 1.44445002]])
b = a[:,:3]
print(b)
print(rotationMatrixToEulerAngles(b))

r = R.from_matrix(b)
print(r.as_quat())
print(a[:,3])