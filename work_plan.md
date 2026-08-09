# ACO-EKF SOC Estimation — Complete Work Plan & Workflow

## What is the Big Picture?

We are building a **smart battery fuel gauge** — a system that always knows
how much charge is left in a lithium-ion battery, even though you cannot
directly measure it.

The system has **two layers working together:**

```
┌─────────────────────────────────────────────────────────────┐
│                      OUTER LAYER: ACO                        │
│   (Ant Colony Optimization — runs ONCE to find best settings)│
│                                                               │
│   ┌─────────────────────────────────────────────────────┐   │
│   │                  INNER LAYER: EKF                    │   │
│   │   (Extended Kalman Filter — runs EVERY second)       │   │
│   │                                                       │   │
│   │   Sensor data → Predict SOC → Correct SOC → Output  │   │
│   └─────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────┘
```

---

## The Complete Pipeline — Step by Step

```
STEP 1: Battery Model
      ↓
STEP 2: Data Generation
      ↓
STEP 3: ACO Optimization (finds best Q and R)
      ↓
STEP 4: EKF with optimized Q and R  →  SOC estimate
      ↓
STEP 5: EKF with default Q and R    →  SOC estimate (baseline)
      ↓
STEP 6: Compare and evaluate
      ↓
STEP 7: Generate plots and figures
```

---

## STEP 1 — Build the Battery Model

**File:** `battery_model.py`

### What is it?
We cannot use the real chemical equations of a battery in a microcontroller —
they are too complex. Instead, we build an **equivalent circuit** that mimics
the battery's electrical behaviour using resistors and capacitors.

### The Circuit (First-Order RC Thevenin Model)

```
         R0 (internal resistance)
  ┌──────/\/\/──────┬──────────────── (+) terminal
  │                 │
OCV(SOC)           R1 ── C1 (polarization branch)
  │                 │
  └─────────────────┴──────────────── (−) terminal
```

| Component | Symbol | Meaning | Value |
|-----------|--------|---------|-------|
| Open circuit voltage | OCV(SOC) | Voltage when no current flows | Depends on SOC |
| Series resistance | R₀ | Immediate voltage drop | 15 mΩ |
| Polarization resistance | R₁ | Slow voltage recovery | 10 mΩ |
| Polarization capacitor | C₁ | Memory effect of battery | 3000 F |
| Capacity | Qₙ | Total charge storage | 2 Ah |

### The Equations

The battery state at each second is:

```
State vector:   x = [SOC,  Vp]
                      ↑      ↑
                 charge   voltage across R1-C1

SOC(k+1) = SOC(k)  −  (η × I × Δt) / Qn     ← current integration
Vp(k+1)  = a × Vp(k)  +  b × I              ← RC dynamics
                                               
Terminal voltage:
  Vt = OCV(SOC) − Vp − R₀ × I               ← what sensor measures
```

### OCV–SOC Relationship

```
OCV (V)
 4.2 ┤                                    ●●●●
     │                               ●●●●●
 4.0 ┤                          ●●●●●
     │                     ●●●●●
 3.8 ┤               ●●●●●●
     │          ●●●●●
 3.6 ┤     ●●●●●
     │ ●●●●
 3.2 ┼──────────────────────────────────────
     0%      20%     40%     60%     80%   100%  SOC
```

> This nonlinear OCV(SOC) curve is why we need an **Extended** Kalman Filter,
> not a plain Kalman Filter.

---

## STEP 2 — Generate Battery Data

**File:** `data_generator.py`

### What is it?
Since we don't have the paper's exact hardware, we simulate a real battery
by running the model with realistic current profiles and adding sensor noise.

### Four Operating Profiles (matching the paper's 4 operating points)

```
Profile 1: CC Discharge (0.5C)
  ─────────────────────────────────
  Current:  ████████████████████  (constant, slow drain)
  SOC:      ▓▓▓▓▓▒▒▒▒▒░░░░░░░░░ (smooth decrease 95% → 70%)
  Duration: 30 minutes

Profile 2: Pulse Discharge (1C)
  ─────────────────────────────────
  Current:  ██░░██░░██░░██░░██░░ (on/off every 30 seconds)
  SOC:      ▓▓▓▓▒▒▒▒░░░░ (stepped decrease)
  Duration: 20 minutes

Profile 3: Dynamic Drive Cycle
  ─────────────────────────────────
  Current:  ██▄▄█░██▄██░░█▄▄█░░ (random bursts, like EV driving)
  SOC:      ▓▓▓▒▒▒░░▒▒░░ (irregular decrease)
  Duration: 30 minutes

Profile 4: CCCV Charge
  ─────────────────────────────────
  Current:  ████████████▄▄▄▄▄▄▄ (full then tapering)
  SOC:      ░░░░░░▒▒▒▒▒▒▓▓▓▓▓▓ (increase 20% → 83%)
  Duration: 87 minutes
```

### Sensor Noise Added

```
True voltage:    ──────────────────── (clean)
Measured voltage: ~~~~~~~~~~~~~~~~~~~~ (±5 mV noise added)

True current:    ──────────────────── (clean)
Measured current: ~~~~~~~~~~~~~~~~~~~~ (±5 mA noise added)
```

### Train/Test Split

```
Profiles 1 & 2  →  TRAINING   (ACO uses these to tune Q and R)
Profiles 3 & 4  →  TESTING    (independent evaluation, unseen by ACO)
```

---

## STEP 3 — EKF: The Real-Time Estimator

**File:** `ekf.py`

### What is it?
The EKF runs **every second** during battery operation. It takes the measured
current and voltage, and outputs a corrected SOC estimate.

### The 7-Step EKF Recursion (one cycle per second)

```
╔══════════════════════════════════════════════════════╗
║           INPUT: Current Ik, Voltage Vt_measured      ║
╠══════════════════════════════════════════════════════╣
║  STEP 1 — Predict SOC                                 ║
║  SOC_predicted = SOC_previous − (η × I × dt) / Qn    ║
║                                                        ║
║  STEP 2 — Predict uncertainty (covariance)            ║
║  P_predicted = F × P × Fᵀ + Q                        ║
║      ↑ Q = how much we trust the battery model        ║
║                                                        ║
║  STEP 3 — Predict terminal voltage                    ║
║  Vt_predicted = OCV(SOC_predicted) − Vp − R₀ × I     ║
║                                                        ║
║  STEP 4 — Calculate voltage error (innovation)        ║
║  error = Vt_measured − Vt_predicted                   ║
║                                                        ║
║  STEP 5 — Calculate Kalman Gain                       ║
║  K = P × Hᵀ × (H × P × Hᵀ + R)⁻¹                   ║
║      ↑ R = how much we trust the voltage sensor       ║
║                                                        ║
║  STEP 6 — Correct SOC                                 ║
║  SOC_corrected = SOC_predicted + K × error            ║
║                                                        ║
║  STEP 7 — Update uncertainty                          ║
║  P_updated = (I − K×H) × P × (I − K×H)ᵀ + K×R×Kᵀ  ║
╠══════════════════════════════════════════════════════╣
║          OUTPUT: SOC_corrected                         ║
╚══════════════════════════════════════════════════════╝
             ↑ Goes back to Step 1 next second
```

### What Q and R Control

```
                        Small R                    Large R
                   (trust sensor more)        (trust sensor less)
        ┌──────────────────────────────────────────────────────┐
Small Q │  Smooth, slow correction.      Very sluggish.         │
(trust  │  Good if model is accurate.   Drifts if model wrong. │
model)  │                                                        │
        ├──────────────────────────────────────────────────────┤
Large Q │  Fast correction.              Very noisy estimates.  │
(doubt  │  Responds quickly to change.  Can become unstable.   │
model)  │                                                        │
        └──────────────────────────────────────────────────────┘
                     ↑ ACO finds the best cell in this table ↑
```

---

## STEP 4 — ACO: The Automatic Tuner

**File:** `aco_optimizer.py`

### What is it?
ACO finds the best values of Q and R automatically, by trying many
combinations and keeping track of which ones work best — just like ants
finding the shortest path to food.

### The ACO Analogy

```
ANTS FINDING FOOD                    ACO FINDING Q AND R
─────────────────────────────────    ──────────────────────────────
Ant explores a path                  ACO tries a Q/R combination
Ant deposits pheromone if path short ACO remembers if SOC error is low
Other ants follow strong pheromone   Next iteration samples near good Q/R
Eventually all ants use best path    Eventually converges to best Q/R
```

### How ACOR (ACO for Continuous domains) Works

```
START
  │
  ▼
Initialize archive of 20 random Q/R candidates
  │
  ▼
┌─────────────────────────────────────────────────────────┐
│  FOR each iteration (30 iterations total):               │
│                                                           │
│    1. Rank archive by SOC error (best = rank 1)          │
│    2. Calculate pheromone weights (rank 1 → most weight) │
│    3. FOR each of 8 ants:                                │
│         a. Pick a kernel from archive (weighted random)   │
│         b. Sample near that kernel (Gaussian noise)       │
│         c. Run EKF with this Q/R on training data        │
│         d. Calculate MSE = mean((SOC_est − SOC_ref)²)    │
│    4. Add 8 new solutions to archive                     │
│    5. Keep only top 20 solutions                         │
│    6. Record best MSE for convergence plot               │
└─────────────────────────────────────────────────────────┘
  │
  ▼
Output: Best Q* and R* found
```

### Search Space (Log Scale)

```
We search in log10 space so ACO can cover a huge range efficiently:

  log10(q_SOC):  −7 to −2   →  q_SOC from 1×10⁻⁷ to 1×10⁻²
  log10(q_Vp):   −7 to −2   →  q_Vp  from 1×10⁻⁷ to 1×10⁻²
  log10(r_V):    −5 to −1   →  r_V   from 1×10⁻⁵ to 1×10⁻¹

Our run found:  Q = diag([1e-7, 1e-7]),  R = 2.455e-3
```

### Objective Function

```
For each Q/R candidate:
  1. Run EKF on training profiles 1 & 2
  2. Compare estimated SOC vs reference SOC
  3. J = (1/N) × Σ (SOC_estimated_k − SOC_reference_k)²   ← minimize this
  4. ACO tries to make J as small as possible
```

---

## STEP 5 — Evaluation & Comparison

**File:** `main.py`

### What happens?

```
┌─────────────────────────────────────────────────┐
│  All 4 profiles run through TWO estimators:      │
│                                                   │
│  Estimator A: Conventional EKF                   │
│               Q = diag([1e-5, 1e-5])  (default) │
│               R = 1e-3               (default)  │
│                                                   │
│  Estimator B: ACO-EKF                            │
│               Q = diag([1e-7, 1e-7])  (ACO-tuned)│
│               R = 2.455e-3            (ACO-tuned)│
│                                                   │
│  Both start with:                                 │
│               Initial SOC = true SOC ± 10%       │
│               (intentional error to test robustness)│
└─────────────────────────────────────────────────┘
```

### Metrics Calculated

| Metric | Formula | What it means |
|--------|---------|---------------|
| **MSE** | mean((est−ref)²) | Overall squared error, penalises large mistakes |
| **MAE** | mean(\|est−ref\|) × 100% | Average absolute error in % SOC |
| **MAPE** | mean(\|err/ref\|) × 100% | Relative error, useful near low SOC |
| **R²** | 1 − SS_res/SS_tot | Closeness to reference (1.0 = perfect) |

---

## STEP 6 — Visualization

**File:** `visualization.py`

### Five Output Figures

```
figures/
├── 01_current_voltage.png   ← What data the battery experienced
├── 02_soc_comparison.png    ← Main result: SOC tracks
├── 03_soc_error.png         ← Error over time (ACO-EKF wins)
├── 04_aco_convergence.png   ← ACO learning curve
└── 05_metrics_table.png     ← Bar chart summary of all metrics
```

---

## Complete Workflow Diagram

```
                  ┌─────────────────────┐
                  │   Battery Hardware   │
                  │  (or simulation)     │
                  └──────────┬──────────┘
                             │ Current I, Voltage Vt
                             ▼
              ┌──────────────────────────┐
              │     data_generator.py    │
              │  4 Operating Profiles:   │
              │  CC / Pulse / Dynamic /  │
              │  CCCV + sensor noise     │
              └──────┬───────────┬───────┘
                     │           │
             Train   │           │  Test
          (P1 & P2)  │           │  (P3 & P4)
                     ▼           │
        ┌────────────────────┐   │
        │  aco_optimizer.py  │   │
        │  ACOR algorithm:   │   │
        │  Try 240 Q/R combos│   │
        │  Keep best 20      │   │
        │  Output Q*, R*     │   │
        └────────┬───────────┘   │
                 │Q*, R*         │
                 ▼               ▼
        ┌─────────────────────────────┐
        │          ekf.py             │
        │   Extended Kalman Filter    │
        │   (runs every second)        │
        │                             │
        │  Uses battery_model.py for: │
        │   • State equations (f)     │
        │   • Voltage equation (h)    │
        │   • Jacobians (F, H)        │
        └──────────────┬──────────────┘
                       │ SOC estimates
                       ▼
        ┌─────────────────────────────┐
        │       visualization.py      │
        │   5 comparison figures      │
        └──────────────┬──────────────┘
                       │
                       ▼
              figures/ folder:
              • SOC comparison plots
              • Error plots
              • ACO convergence
              • Metrics bar charts
```

---

## Summary: What Each File Does

| File | Role | When it runs |
|------|------|-------------|
| `battery_model.py` | Defines the physics of the battery | Always (imported by others) |
| `data_generator.py` | Creates synthetic test data with 4 profiles | Once at startup |
| `ekf.py` | The real-time SOC estimator | Every sample during evaluation |
| `aco_optimizer.py` | Finds the best Q and R settings | Once (offline tuning phase) |
| `visualization.py` | Draws all comparison plots | Once at end |
| `main.py` | Ties everything together | You run this |
| `README.md` | Documentation | For reference |

---

## What the Paper Contributes (vs a Standard EKF)

```
Standard EKF approach:
  Engineer manually guesses Q and R
       ↓
  Tests EKF
       ↓
  Adjusts Q and R by trial and error  ← TIME CONSUMING, SUBJECTIVE
       ↓
  Repeats until satisfied

ACO-EKF approach (this paper):
  ACO automatically searches Q and R space
       ↓
  ACO evaluates 240+ combinations objectively
       ↓
  ACO finds Q* and R* that minimize SOC error  ← AUTOMATIC, OBJECTIVE
       ↓
  EKF runs with optimal parameters
```

---

## Our Results vs Paper

| Metric | Paper Results | Our Simulation |
|--------|--------------|---------------|
| MSE | 0.0165% | ~0 (synthetic, no model mismatch) |
| MAE | 1.2% | **0.024–0.035%** |
| MAPE | 2% | **0.028–0.068%** |
| R² | 99.34% | **99.98–99.99%** |
| ACO-EKF vs Conv. EKF | Better | **2–3× lower error** ✅ |

> [!NOTE]
> Our numbers are better than the paper because we simulate a perfect battery
> (no model mismatch, no aging, no temperature). The paper uses real hardware
> where all those imperfections exist. The **relative improvement of ACO-EKF
> over conventional EKF is the key finding** — and we confirm that exactly.
