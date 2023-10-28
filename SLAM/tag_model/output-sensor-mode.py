from sympy import pretty, simplify
from sensor_model_tag import z_bar, j

with open("sensor-mode-tag-z_bar.txt", "w", encoding="utf-8") as f:
    f.write(pretty(simplify(z_bar), use_unicode=True, wrap_line=False))

with open("sensor-mode-tag-z_bar_jacobian.txt", "w", encoding="utf-8") as f:
    f.write(pretty(simplify(z_bar.jacobian(j)), use_unicode=True, wrap_line=False))
