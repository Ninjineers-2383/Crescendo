from sympy import pretty, simplify, MatrixSymbol
from sympy.printing import ccode
from motion_model import mu_d, mu_t
import re

mu_text = pretty(simplify(mu_d), use_unicode=True, wrap_line=False)
mu_jacob_text = pretty(simplify(mu_d.jacobian(mu_t)), use_unicode=True, wrap_line=False)

with open("motion-model-mu.txt", "w", encoding="utf-8") as f:
    f.write(mu_text)

with open("motion-model-mu_jacobian.txt", "w", encoding="utf-8") as f:
    f.write(mu_jacob_text)
java_template = """package com.team2383.lib.SLAM;

import java.util.function.BiFunction;

import org.ejml.simple.SimpleMatrix;

public class {0} implements BiFunction<SimpleMatrix, SimpleMatrix, SimpleMatrix> {{
    private SimpleMatrix F_x;

    public {0}(SimpleMatrix F_x) {{
        this.F_x = F_x;
    }}

    @Override
    public SimpleMatrix apply(SimpleMatrix u, SimpleMatrix mu) {{
        double dx = u.get(0);
        double dy = u.get(1);
        double dtheta = u.get(2);

        double rw = mu.get(3);
        double rx = mu.get(4);
        double ry = mu.get(5);
        double rz = mu.get(6);

        SimpleMatrix {1} = new SimpleMatrix({2}, {3});

{4}

        return F_x.transpose().mult({1}){5};
    }}
}}
"""


def convert_to_java(string):
    string = re.sub(
        r"\[(?P<idx>\d*)\] = (?P<val>.*);", r".set(\g<idx>, \g<val>);", string
    )

    string = string.replace("pow", "Math.pow")
    string = string.replace("sqrt", "Math.sqrt")
    string = string.replace("sin", "Math.sin")
    string = string.replace("cos", "Math.cos")

    return string


g = MatrixSymbol("g", 7, 1)

G = MatrixSymbol("G", 7, 7)

with open("MotionModel.java", "w") as f:
    f.write(
        java_template.format(
            "MotionModel", "g", "7", "1", convert_to_java(ccode(simplify(mu_d), g)), ""
        )
    )

with open("MotionModelJacobian.java", "w") as f:
    f.write(
        java_template.format(
            "MotionModelJacobian",
            "G",
            "7",
            "7",
            convert_to_java(ccode(simplify(mu_d.jacobian(mu_t)), G)),
            ".mult(F_x)",
        )
    )
