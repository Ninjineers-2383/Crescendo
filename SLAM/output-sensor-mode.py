from sympy import pretty, simplify
from sensor_model_tag import z_bar, j

with open("sensor-mode-tag-z_bar.txt", "w") as f:
    f.write(pretty(z_bar, use_unicode=False, wrap_line=False))

with open("sensor-mode-tag-z_bar_jacobian.txt", "w") as f:
    f.write(pretty(z_bar.jacobian(j), use_unicode=False, wrap_line=False))
