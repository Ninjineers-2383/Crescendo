from sensor_model_tag import *

setup = {
    x: 0,
    y: 0,
    z: 0,
    alpha: 0,
    beta: 0,
    theta: 0,
    jx: 1,
    jy: 1,
    jz: 1,
    jalpha: 0,
    jbeta: 0,
    jtheta: 0,
}

setup2 = {
    x: 0,
    y: 1,
    z: 2,
    alpha: 3,
    beta: 1.2,
    theta: -1,
    jx: 2,
    jy: 5,
    jz: 8,
    jalpha: 0.12,
    jbeta: 0.4,
    jtheta: 0.01,
}

pprint(z_bar.subs(setup2).evalf(5))
