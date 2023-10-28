from sympy import pretty, simplify
from motion_model import mu, mu_t

mu_text = pretty(simplify(mu), use_unicode=True, wrap_line=False)
mu_jacob_text = pretty(simplify(mu.jacobian(mu_t)), use_unicode=True, wrap_line=False)

with open("motion-model-mu.txt", "w", encoding="utf-8") as f:
    f.write(mu_text)

with open("motion-model-mu_jacobian.txt", "w", encoding="utf-8") as f:
    f.write(mu_jacob_text)

print(mu_text)
print(mu_jacob_text)
