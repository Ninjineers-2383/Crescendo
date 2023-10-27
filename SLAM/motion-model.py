from sympy import symbols, Matrix, cos, sin, init_printing, pprint

init_printing()

# Define symbols
x, y, z, alpha, beta, theta = symbols("x y z alpha beta theta")

dx, dy, dtheta = symbols("dx dy dtheta")

# Define motion model

# Odometry motion model
mu_t = Matrix([x, y, z, alpha, beta, theta])

mu = mu_t + Matrix(
    [
        dx * cos(theta) - dy * sin(theta),
        dx * sin(theta) + dy * cos(theta),
        0,
        0,
        0,
        dtheta,
    ]
)

pprint(mu.jacobian(mu_t))
