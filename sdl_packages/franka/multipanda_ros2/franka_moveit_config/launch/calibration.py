import numpy as np

# ---- Input: 4x4 Transform Matrix ----
T = np.array([
    [0.00386347, -0.998878,    0.047198,   0.0146904],
    [0.999511,    0.00532242,  0.0308248, -0.0198886],
    [-0.0310415,  0.0470558,   0.99841,   -0.0641156],
    [0,          0,           0,          1]
])

# ---- Extract translation ----
translation = T[:3, 3]

# ---- Extract rotation (3x3) ----
R = T[:3, :3]

# ---- Convert rotation matrix to quaternion ----
# quaternion in (x, y, z, w) format
trace = np.trace(R)
if trace > 0:
    s = 0.5 / np.sqrt(trace + 1.0)
    w = 0.25 / s
    x = (R[2,1] - R[1,2]) * s
    y = (R[0,2] - R[2,0]) * s
    z = (R[1,0] - R[0,1]) * s
else:
    if (R[0,0] > R[1,1]) and (R[0,0] > R[2,2]):
        s = 2.0 * np.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2])
        w = (R[2,1] - R[1,2]) / s
        x = 0.25 * s
        y = (R[0,1] + R[1,0]) / s
        z = (R[0,2] + R[2,0]) / s
    elif R[1,1] > R[2,2]:
        s = 2.0 * np.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2])
        w = (R[0,2] - R[2,0]) / s
        x = (R[0,1] + R[1,0]) / s
        y = 0.25 * s
        z = (R[1,2] + R[2,1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1])
        w = (R[1,0] - R[0,1]) / s
        x = (R[0,2] + R[2,0]) / s
        y = (R[1,2] + R[2,1]) / s
        z = 0.25 * s

quat = np.array([x, y, z, w])

print("Translation ('%f', '%f', '%f'):" % (translation[0], translation[1], translation[2]))

print("\nQuaternion ('%f', '%f', '%f', '%f'):" % (quat[0], quat[1], quat[2], quat[3]))
