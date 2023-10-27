from sympy import (
    symbols,
    Matrix,
    cos,
    sin,
    atan2,
    sqrt,
    init_printing,
)

init_printing()

# Define symbols
# Robot pose
x, y, z, alpha, beta, theta = symbols("x y z alpha beta theta")
mu = Matrix([x, y, z, alpha, beta, theta])

# Estimated Tag Pose
jx, jy, jz, jalpha, jbeta, jtheta = symbols("jx jy jz jalpha jbeta jtheta")
j = Matrix([jx, jy, jz, jalpha, jbeta, jtheta])

# Measured Tag Pose
zx, zy, zz, zalpha, zbeta, ztheta = symbols("zx zy zz zalpha zbeta ztheta")
z_r = Matrix([zx, zy, zz, zalpha, zbeta, ztheta])

# Define rotation matrices
rotZ = Matrix(
    [
        [cos(theta), -sin(theta), 0],
        [sin(theta), cos(theta), 0],
        [0, 0, 1],
    ]
)

rotY = Matrix(
    [
        [cos(beta), 0, sin(beta)],
        [0, 1, 0],
        [-sin(beta), 0, cos(beta)],
    ]
)

rotX = Matrix(
    [
        [1, 0, 0],
        [0, cos(alpha), -sin(alpha)],
        [0, sin(alpha), cos(alpha)],
    ]
)

fullRot = rotZ * rotY * rotX

z_bar_tran = fullRot.transpose() * (j - mu)[0:3, 0]

# subtract the tag rotation from the robot rotation
z_bar_rot_mat = fullRot.transpose() * fullRot.subs(alpha, jalpha).subs(
    beta, jbeta
).subs(theta, jtheta)

dbeta = atan2(
    -z_bar_rot_mat[2, 0], sqrt((z_bar_rot_mat[2, 1] ** 2) + (z_bar_rot_mat[2, 2] ** 2))
)

dalpha = atan2(z_bar_rot_mat[1, 0] / cos(dbeta), z_bar_rot_mat[0, 0] / cos(dbeta))
dtheta = atan2(z_bar_rot_mat[2, 1] / cos(dbeta), z_bar_rot_mat[2, 2] / cos(dbeta))

z_bar = Matrix([z_bar_tran[0], z_bar_tran[1], z_bar_tran[2], dtheta, dbeta, dalpha])
