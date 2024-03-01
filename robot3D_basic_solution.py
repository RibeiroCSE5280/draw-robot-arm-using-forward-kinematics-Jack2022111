from robot3D_basic import getLocalFrameMatrix, RotationMatrix
import numpy as np


def forward_kinematics(Phi, L1, L2, L3, L4):
    # Diameter of the spheres
    diameter = 0.8
    phi1, phi2, phi3, phi4 = Phi

    # Matrix of Frame 1 (written w.r.t. Frame 0, which is the previous frame)
    R_01 = RotationMatrix(phi1, axis_name='z')  # Rotation matrix
    p1 = np.array([[3], [2], [0.0]])  # Frame's origin (w.r.t. previous frame)
    t_01 = p1  # Translation vector

    # Matrix of Frame 1 w.r.t. Frame 0 (i.e., the world frame)
    T_01 = getLocalFrameMatrix(R_01, t_01)

    # Matrix of Frame 2 (written w.r.t. Frame 1, which is the previous frame)
    R_12 = RotationMatrix(phi2, axis_name='z')  # Rotation matrix
    p2 = np.array([[L1 + diameter], [0.0], [0.0]])  # Frame's origin (w.r.t. previous frame)
    t_12 = p2  # Translation vector

    # Matrix of Frame 2 w.r.t. Frame 1
    T_12 = getLocalFrameMatrix(R_12, t_12)

    # Matrix of Frame 2 w.r.t. Frame 0 (i.e., the world frame)
    T_02 = T_01 @ T_12

    # Matrix of Frame 3 (written w.r.t. Frame 2, which is the previous frame)
    R_23 = RotationMatrix(phi3, axis_name='z')  # Rotation matrix
    t_23 = np.array([[L2 + diameter], [0.0], [0.0]])

    # Matrix of Frame 3 w.r.t. Frame 2
    T_23 = getLocalFrameMatrix(R_23, t_23)

    # Matrix of Frame 3 w.r.t. Frame 0 (i.e., the world frame)
    T_03 = T_01 @ T_12 @ T_23

    # Matrix of Frame 3 (written w.r.t. Frame 2, which is the previous frame)
    R_34 = RotationMatrix(phi3, axis_name='z')  # Rotation matrix
    p3 = np.array([[L3 + diameter / 2], [0.0], [0.0]])
    t_34 = p3  # Translation vector

    # Matrix of end-effector w.r.t. Frame 3
    T_34 = getLocalFrameMatrix(R_34, t_34)

    T_04 = T_03 @ T_34
    e = T_04[:, -1][:3]

    return T_01, T_02, T_03, T_04, e

