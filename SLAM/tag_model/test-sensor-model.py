from sensor_model_tag import (
    x,
    y,
    z,
    rw,
    rx,
    ry,
    rz,
    jx,
    jy,
    jz,
    jrw,
    jrx,
    jry,
    jrz,
    z_bar,
)
from sympy import pprint

setup = {
    x: 0,
    y: 0,
    z: 0,
    rw: 1,
    rx: 0,
    ry: 0,
    rz: 0,
    jx: 1,
    jy: 1,
    jz: 1,
    jrw: 1,
    jrx: 0,
    jry: 0,
    jrz: 0,
}

setup2 = {
    x: 0,
    y: 1,
    z: 2,
    rw: -0.21879093941674016,
    rx: 0.7416346067812737,
    ry: -0.359644044901111,
    rz: -0.5222688922582828,
    jx: 2,
    jy: 5,
    jz: 8,
    jrw: 0.9783503232087202,
    jrx: 0.05777642898165732,
    jry: 0.19860319674421734,
    jrz: -0.007021365572178099,
}

pprint(z_bar.subs(setup2).evalf(5))
