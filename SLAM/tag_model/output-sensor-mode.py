from sympy import pretty, simplify, Matrix, MatrixSymbol
from sympy.printing import ccode
from sensor_model_tag import z_bar, mu, j
import re

state = Matrix([mu, j])

z_bar_simplified = simplify(z_bar)
z_bar_simplified_jacob = simplify(z_bar.jacobian(state))

with open("sensor-mode-tag-z_bar.txt", "w", encoding="utf-8") as f:
    f.write(pretty(z_bar_simplified, use_unicode=True, wrap_line=False))

with open("sensor-mode-tag-z_bar_jacobian.txt", "w", encoding="utf-8") as f:
    f.write(pretty(z_bar_simplified_jacob, use_unicode=True, wrap_line=False))

java_template = """package com.team2383.lib.SLAM;

import java.util.function.BiFunction;

import org.ejml.simple.SimpleMatrix;

public class {0} implements BiFunction<SimpleMatrix, SimpleMatrix, SimpleMatrix> {{
    @Override
    public SimpleMatrix apply(SimpleMatrix mu, SimpleMatrix j_t) {{
        double x = mu.get(0);
        double y = mu.get(1);
        double z = mu.get(2);

        double rw = mu.get(3);
        double rx = mu.get(4);
        double ry = mu.get(5);
        double rz = mu.get(6);

        int j = (int)j_t.get(0);
        int offset = 7 * (j + 1);
        double jx = mu.get(0 + offset);
        double jy = mu.get(1 + offset);
        double jz = mu.get(2 + offset);

        double jrw = mu.get(3 + offset);
        double jrx = mu.get(4 + offset);
        double jry = mu.get(5 + offset);
        double jrz = mu.get(6 + offset);

        SimpleMatrix {1} = new SimpleMatrix({2}, {3});

{4}

        return {1};
    }}
}}
"""


def convert_to_java(string):
    string = re.sub(
        r"\[(?P<idx>\d*)\] = (?P<val>.*);", r".set(\g<idx>, \g<val>);", string
    )

    string = string.replace("pow", "Math.pow")
    string = string.replace("sqrt", "Math.sqrt")

    return string


h = MatrixSymbol("h", 7, 1)

H = MatrixSymbol("H", 7, 14)

with open("SensorModelTag.java", "w") as f:
    f.write(
        java_template.format(
            "SensorModelTag", "h", "7", "1", convert_to_java(ccode(z_bar_simplified, h))
        )
    )

with open("SensorModelTagJacobian.java", "w") as f:
    f.write(
        java_template.format(
            "SensorModelTagJacobian",
            "H",
            "7",
            "14",
            convert_to_java(ccode(z_bar_simplified_jacob, H)),
        )
    )
