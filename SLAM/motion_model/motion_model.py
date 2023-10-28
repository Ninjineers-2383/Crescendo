from sympy import symbols, Matrix, cos, sin, init_printing, Quaternion

init_printing()

# Define symbols
x, y, z, rw, rx, ry, rz = symbols("x y z rw rx ry rz")

dx, dy, dtheta = symbols("dx dy dtheta")

# Define motion model

# Odometry motion model
mu_t = Matrix([x, y, z, rw, rx, ry, rz])

motion_command = Quaternion(0, dx, dy, 0)

motion = Quaternion(rw, rx, ry, rz) * motion_command * Quaternion(rw, -rx, -ry, -rz)

rotation = Quaternion(cos(dtheta / 2), 0, 0, sin(dtheta / 2))

rotation_diff = rotation * Quaternion(rw, rx, ry, rz)


mu = mu_t + Matrix(
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
