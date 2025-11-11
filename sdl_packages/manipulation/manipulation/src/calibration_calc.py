import numpy as np
from scipy.spatial.transform import Rotation as R

# ---- 4x4 transform matrix ----
T = np.array([
    [0.037698,   -0.998655,   0.0355989,  0.0306668],
    [0.999281,    0.0375304, -0.00536369, -0.0332122],
    [0.00402043,  0.0357755,  0.999352,  -0.0594572],
    [0.0,         0.0,         0.0,        1.0]
])

# ---- Extract translation ----
translation = T[:3, 3]

# ---- Extract rotation matrix ----
Rmat = T[:3, :3]

# ---- Convert to quaternion (x,y,z,w) ----
quat = R.from_matrix(Rmat).as_quat()

print("Translation ('x', 'y', 'z'):")
print(translation)

print("\nQuaternion ('x', 'y', 'z', 'w'):")
print(quat)
print("'%.8f', '%.8f', '%.8f', '%.8f', '%.8f', '%.8f', '%.8f'" % 
      (translation[0], translation[1], translation[2],
       quat[0], quat[1], quat[2], quat[3]))
