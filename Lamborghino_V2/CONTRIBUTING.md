# ⚠️ Important Notes for Contributors

This project simulates a robot model based on the real Open Lamborghino platform, with a front caster ball (rotating sphere) replacing the original pivot, positioned behind the ground sensors.

Please consider the following constraints before modifying the controller or robot behavior:

## Physical & Model Constraints

- The robot does not use suction or aerodynamic downforce
- The front support is a caster ball
- Abrupt speed changes may cause physical instability
- Excessive acceleration can result in:
  - Front caster ball lift
  - Reduced or lost ground sensor contact
  - Track departure in tight or high-curvature segments
These behaviors intentionally reflect limitations of lightweight real-world robotracers.

---

# 🚦 Speed and Acceleration Policy

The progressive speed control strategy implemented in this project is intentional and required for stability.

When contributing, please observe the following guidelines:
- ❌ Avoid instantaneous speed changes
- ✅ Preserve gradual acceleration and deceleration logic
- ✅ Respect dynamic speed limits tied to track curvature
- ⚠️ Test carefully any modification that increases target or limit speeds

All speed-related changes must be validated under:
- Sharp curves
- Rapid direction changes
- High-curvature track segments

---

# ✅ Contribution Philosophy

Contributions that:
- Improve lap time
- Enhance control robustness
- Reduce oscillations
- Maintain or improve sensor stability

without compromising physical realism or stability are especially welcome.

Performance gains achieved through smoother control and better anticipation, rather than brute-force speed increases, are strongly preferred.

---

# 🧠 Why This Matters

This repository is not only a simulation project, but also a control-systems learning platform that prioritizes:
- Deterministic behavior
- Realistic constraints
- Transferable control strategies

Please keep these goals in mind when proposing changes.
