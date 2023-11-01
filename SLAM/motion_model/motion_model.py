from sympy import symbols, Matrix, cos, sin, init_printing, Quaternion, sqrt

init_printing()

# Define symbols
x, y, z, rw, rx, ry, rz = symbols("x y z rw rx ry rz")

dx, dy, dtheta = symbols("dx dy dtheta")

# Define motion model

# Odometry motion model
mu_t = Matrix([x, y, z, rw, rx, ry, rz])

motion_command = Quaternion(0, dx, dy, 0)

robot_rot = Quaternion(rw, rx, ry, rz).normalize()

motion = robot_rot * motion_command * robot_rot.conjugate()

rotation = Quaternion(cos(dtheta / 2), 0, 0, sin(dtheta / 2))

rotation_diff = rotation * Quaternion(
    rw / sqrt(rw**2 + rz**2), 0, 0, rz / sqrt(rw**2 + rz**2)
)


mu_d = Matrix(
    [
        motion.b,
        motion.c,
        motion.d,
        rotation_diff.a - rw,
        rotation_diff.b - rx,
        rotation_diff.c - ry,
        rotation_diff.d - rz,
    ]
)
