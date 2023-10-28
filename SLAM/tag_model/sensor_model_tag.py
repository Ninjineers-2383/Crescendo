from sympy import (
    symbols,
    Matrix,
    init_printing,
    Quaternion,
)

init_printing()

# Define symbols
# Robot pose
x, y, z, rw, rx, ry, rz = symbols("x y z rw rx ry rz")
mu = Matrix([x, y, z, rw, rx, ry, rz])

# Estimated Tag Pose
jx, jy, jz, jrw, jrx, jry, jrz = symbols("jx jy jz jrw jrx jry jrz")
j = Matrix([jx, jy, jz, jrw, jrx, jry, jrz])

# Measured Tag Pose
zx, zy, zz, zrw, zrx, zry, zrz = symbols("zx zy zz zrw zrx zry zrz")
z_r = Matrix([zx, zy, zz, zrw, zrx, zry, zrz])

robot_rot = Quaternion(rw, rx, ry, rz)

displacement = (j - mu)[0:3, 0]

z_bar_tran_quat = (
    robot_rot.inverse()
    * Quaternion(0, displacement[0], displacement[1], displacement[2])
    * robot_rot
)

z_bar_tran = Matrix([z_bar_tran_quat.b, z_bar_tran_quat.c, z_bar_tran_quat.d])

# subtract the tag rotation from the robot rotation
z_bar_rot = robot_rot.inverse() * Quaternion(jrw, jrx, jry, jrz)


z_bar = Matrix(
    [
        z_bar_tran[0],
        z_bar_tran[1],
        z_bar_tran[2],
        z_bar_rot.a,
        z_bar_rot.b,
        z_bar_rot.c,
        z_bar_rot.d,
    ]
)
