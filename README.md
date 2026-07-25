# Multi-Objective ACO-Tuned Extended Kalman Filter for Li-ion SoC Estimation

Estimating the State of Charge (SoC) of a lithium-ion battery using an
Extended Kalman Filter (EKF) built on an R-2RC equivalent circuit model —
and automatically tuning the EKF's process/measurement noise covariances
(Q, R) using **Multi-Objective Ant Colony Optimization (MOACO)**, instead
of the single-objective (accuracy-only) tuning used in prior work.

Base paper: *"Ant Colony Optimized Extended Kalman Filter for State of
Charge Estimation of Lithium-Ion Batteries,"* IEEE Trans. Instrumentation
and Measurement, vol. 74, 2025.

## Problem

SoC can't be measured directly — it must be estimated from voltage and
current. EKF is the most reliable model-based method, but its accuracy
depends entirely on two hand-tuned parameters, Q and R. Every prior
tuning method (including the base paper's own ACO-EKF) optimizes these
for accuracy alone.

## Our contribution

We reformulate Q/R tuning as a **multi-objective** problem:

```
Existing:  Minimize  SSE = f(Q, R)
Ours:      Minimize  [ SSE ,  T_conv ,  C_comp ]
```

optimizing accuracy (SSE), convergence speed (T_conv), and computational
cost (C_comp) together — producing a **Pareto front** of (Q, R)
trade-offs instead of one fixed answer.

## Repository structure

```
aco-ekf-soc-estimation/
├── models/            SoC-dependent battery parameter equations (R-2RC)
│   ├── battery_params.m
│   └── ocv_derivative.m
├── simulation/         Week 1: plant model + discharge profile generation
│   ├── generate_profile.m
│   ├── simulate_battery.m
│   └── main_week1.m
├── ekf/                Week 2: baseline EKF (predict-correct)          [in progress]
├── aco/                Weeks 3-4: single-objective ACO + MOACO         [in progress]
├── results/plots/       Generated figures (Pareto fronts, comparisons)
└── docs/                Project reference doc, review slides
```

## Roadmap (5-week simulation plan)

| Week | Deliverable | Status |
|------|-------------|--------|
| 1 | R-2RC battery model, validated across 4 discharge profiles | ✅ Done |
| 2 | Baseline EKF, validated against Coulomb Counting | ⬜ In progress |
| 3 | Reproduce base paper's single-objective ACO result (Q≈2.7e-6, R≈0.044) | ⬜ Planned |
| 4 | Multi-Objective ACO (weighted-sum) across all 4 profiles | ⬜ Planned |
| 5 | Pareto front analysis + final comparison vs. baseline | ⬜ Planned |

## How to run (Week 1)

```matlab
% From MATLAB, with this repo as your current folder:
cd simulation
main_week1
```

This runs all four discharge profiles (constant, pulse, dual-pulse, DC
motor load) through the R-2RC model and saves a validation plot to
`results/plots/`.

## Tools

MATLAB (simulation, EKF, ACO/MOACO development). Reference battery: 12 Ah,
25.6 V LiFePO4 pack (see base paper, Table III).

## License

MIT — see [LICENSE](LICENSE).
