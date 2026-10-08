# Professional Project Blueprint: ACO-Optimized EKF for Lithium-Ion Battery SoC Estimation

## 1. Project identity

**Proposed title:** Ant Colony Optimization–Tuned Extended Kalman Filter for Real-Time State-of-Charge Estimation of Lithium-Ion Batteries

**Project type:** Embedded battery-management and nonlinear state-estimation project.

**Base paper:** *Ant Colony Optimized Extended Kalman Filter for State of Charge Estimation of Lithium-Ion Batteries.* The paper implements an ACO-tuned EKF on a low-cost microcontroller and optimizes the EKF process and measurement covariance parameters, Q and R [^1].

## 2. Motivation and objective

A battery-management system cannot directly measure SoC with a simple sensor. It must infer SoC from current, terminal voltage, temperature, and a battery model. An inaccurate estimate can cause premature shutdown, over-discharge, over-charge, poor energy utilization, and unsafe operation [^1].

The central problem is that a fixed EKF tuning is rarely optimal across different current profiles, temperatures, sensors, and battery ageing states. The project therefore combines:

1. an equivalent-circuit model to describe battery dynamics;
2. an EKF to estimate the hidden states; and
3. ACO to search for Q and R values that minimize SoC estimation error.

**Primary objective:** Develop and experimentally validate a real-time estimator that produces accurate, stable, and computationally feasible SoC estimates under constant, pulsed, and variable-current operation.

**Secondary objectives:**
- identify the electrical parameters of an R–2RC battery model;
- implement the EKF in MATLAB/Python first and then on an embedded controller;
- compare Coulomb counting, conventional EKF, and ACO-EKF;
- quantify accuracy, convergence, computation time, and robustness;
- document failure cases caused by temperature, ageing, sensor bias, and model mismatch.

## 3. System-level design

```text
Battery pack
   ├── Current sensor ── signal conditioning ── ADC ┐
   ├── Voltage sensor ── divider/conditioning ──────┤
   └── Temperature sensor ──────────────────────────┤
                                                     v
                                             Microcontroller
                                      ┌────────────────────────┐
                                      │ Sampling and filtering  │
                                      │ R–2RC battery model     │
                                      │ EKF prediction/update   │
                                      │ ACO-tuned Q and R      │
                                      │ SoC bounds and logging  │
                                      └───────────┬────────────┘
                                                  ├── SoC display/logging
                                                  ├── communication interface
                                                  └── protection/control output
```

The base paper uses a LiFePO4 pack, Hall-effect current sensing, a voltage divider, ADC acquisition, signal conditioning, a microcontroller, relay/load control, temperature sensing, and Bluetooth logging [^1]. The proposed project should retain this architecture but add explicit temperature logging, sensor calibration, fault flags, watchdog protection, and data timestamping.

## 4. Battery model

Use a Thevenin R–2RC equivalent-circuit model. The states are:

$$x_k = \begin{bmatrix} z_k \\ V_{p,k} \\ V_{c,k} \end{bmatrix}$$

where $$z_k$$ is SoC, while $$V_{p,k}$$ and $$V_{c,k}$$ are polarization voltages of the two RC branches. The input is battery current $$I_k$$, and the measured output is terminal voltage $$V_{t,k}$$. This is the same three-state structure used by the base paper [^1].

Adopt the sign convention that positive current means discharge. Then the continuous-time model is:

$$\dot z(t) = -\frac{\eta I(t)}{Q_n}$$

$$\dot V_p(t) = -\frac{V_p(t)}{R_p(z,T)C_p(z,T)} + \frac{I(t)}{C_p(z,T)}$$

$$\dot V_c(t) = -\frac{V_c(t)}{R_c(z,T)C_c(z,T)} + \frac{I(t)}{C_c(z,T)}$$

$$V_t(t) = OCV(z,T) - I(t)R_i(z,T) - V_p(t) - V_c(t) + v(t)$$

Here, $$Q_n$$ is nominal capacity in coulombs, $$\eta$$ is Coulombic efficiency, $$R_i$$ is ohmic resistance, and $$R_p,C_p,R_c,C_c$$ describe transient polarization branches.

For sampling period $$\Delta t$$, define:

$$a_p = \exp\left(-\frac{\Delta t}{R_pC_p}\right), \qquad a_c = \exp\left(-\frac{\Delta t}{R_cC_c}\right)$$

$$b_p = R_p(1-a_p), \qquad b_c = R_c(1-a_c)$$

The discrete state equation becomes:

$$x_{k+1}=f(x_k,I_k)+w_k$$

$$z_{k+1}=z_k-\frac{\eta\Delta t}{Q_n}I_k$$

$$V_{p,k+1}=a_pV_{p,k}+b_pI_k$$

$$V_{c,k+1}=a_cV_{c,k}+b_cI_k$$

The nonlinear measurement equation is:

$$y_k=h(x_k,I_k)=OCV(z_k)-I_kR_i(z_k)-V_{p,k}-V_{c,k}+v_k$$

The base paper represents OCV with a third-order SoC polynomial, for example:

$$OCV(z)=0.5214z^3-0.9611z^2+0.66z+3.13$$

but the project should identify its own OCV curve for the selected cell chemistry and temperature rather than blindly reusing this polynomial [^1].

## 5. Parameter identification

Before EKF operation, identify $$R_i,R_p,R_c,C_p,C_c$$ as functions of SoC and, preferably, temperature. Use low-current characterization and pulse-current experiments:

- fully charge the cell using the manufacturer's limits;
- allow a rest period;
- apply controlled current pulses at several SoC points;
- record current, voltage, temperature, and time;
- estimate the instantaneous voltage drop for $$R_i$$;
- fit the relaxation response to obtain the two RC time constants;
- interpolate or fit each parameter versus SoC and temperature.

The paper uses interpolation and curve fitting so the model parameters vary with SoC [^1]. The major improvement for this project is to prevent temperature and ageing from being hidden inside a single fixed parameter set.

## 6. EKF controller/estimator design

This is not a conventional feedback controller that directly regulates motor speed or voltage. It is a **state-estimation controller inside the BMS**: it uses measured current and voltage to generate a corrected internal estimate of SoC, which can then drive charge/discharge protection and supervisory decisions.

For the nonlinear model:

$$x_{k+1}=f(x_k,u_k)+w_k, \qquad y_k=h(x_k,u_k)+v_k$$

where $$u_k=I_k$$, $$w_k\sim\mathcal{N}(0,Q)$$, and $$v_k\sim\mathcal{N}(0,R)$$.

### EKF prediction

$$\hat{x}_{k|k-1}=f(\hat{x}_{k-1|k-1},u_{k-1})$$

$$F_k=\left.\frac{\partial f}{\partial x}\right|_{\hat{x}_{k-1|k-1},u_{k-1}}$$

$$P_{k|k-1}=F_kP_{k-1|k-1}F_k^T+Q$$

### EKF measurement correction

$$\hat{y}_k=h(\hat{x}_{k|k-1},u_k)$$

$$H_k=\left.\frac{\partial h}{\partial x}\right|_{\hat{x}_{k|k-1},u_k}$$

$$S_k=H_kP_{k|k-1}H_k^T+R$$

$$K_k=P_{k|k-1}H_k^TS_k^{-1}$$

$$r_k=y_k-\hat{y}_k$$

$$\hat{x}_{k|k}=\hat{x}_{k|k-1}+K_kr_k$$

Use the numerically safer Joseph covariance update:

$$P_{k|k}=(I-K_kH_k)P_{k|k-1}(I-K_kH_k)^T+K_kRK_k^T$$

After each update, enforce physical bounds:

$$0\leq \hat z_k\leq 1$$

and reject or flag impossible measurements, such as a voltage outside the calibrated sensor range.

For the R–2RC model, the measurement Jacobian is:

$$H_k=\begin{bmatrix}\frac{dOCV}{dz}-I_k\frac{dR_i}{dz} & -1 & -1\end{bmatrix}$$

when the parameter dependence on SoC is included. If the resistance is treated as locally constant during one update, use $$H_k=[dOCV/dz,-1,-1]$$.

## 7. ACO tuning approach

The ACO layer is used primarily offline or during commissioning. It should not run continuously on a small microcontroller unless its computational cost has been demonstrated to be safe.

Represent each ant as a candidate covariance pair:

$$\theta_j=[q_j,r_j]$$

with diagonal process covariance:

$$Q=qI_3$$

and scalar measurement covariance:

$$R=r$$

The base paper searches approximately over:

$$10^{-8}\leq q\leq10^{-5}, \qquad 0.001\leq r\leq0.08$$

and defines pheromone quality from the SoC sum-squared error [^1].

For each candidate pair, run the complete EKF over a calibration data set and calculate:

$$J(q,r)=\sum_{k=1}^{N}(z_{ref,k}-\hat z_k)^2$$

A more professional objective should penalize both accuracy and undesirable estimator behavior:

$$J=\lambda_1RMSE_z+\lambda_2MAE_z+\lambda_3E_{\%}+\lambda_4T_{conv}+\lambda_5N_{viol}$$

where $$E_{\%}$$ is percentage error, $$T_{conv}$$ is convergence time, and $$N_{viol}$$ counts bound violations or unstable covariance events.

Pheromone quality can be defined as:

$$\phi_j=\frac{1}{\epsilon+J_j}$$

At each iteration:

1. randomly initialize ants within valid bounds;
2. evaluate each pair by running the EKF;
3. identify the lowest-cost ant;
4. move the remaining ants toward the best candidate with controlled random exploration;
5. clip candidates to the valid Q/R bounds;
6. retain the global best candidate;
7. stop when the improvement remains below a tolerance for several iterations;
8. validate the selected pair on data not used for tuning.

The paper reports repeated independent ACO runs and a systematic decrease in best error across iterations [^1]. Your project should additionally report run-to-run variation and validation-set performance, not only the best training error.

## 8. Experimental plan

**Stage 1 — simulation:** Validate the R–2RC model and EKF using synthetic data with known noise and known SoC.

**Stage 2 — cell characterization:** Generate OCV–SoC and pulse-response data at controlled temperature.

**Stage 3 — offline estimation:** Compare Coulomb counting, fixed-Q/R EKF, manually tuned EKF, and ACO-EKF.

**Stage 4 — hardware-in-the-loop:** Replay recorded current profiles into the embedded estimator and compare real-time output with the offline implementation.

**Stage 5 — live battery test:** Test constant-current, pulsed-current, and mixed-current profiles at safe limits.

**Stage 6 — robustness tests:** Repeat at multiple initial SoCs, temperatures, sensor-noise levels, and ageing conditions.

Use separate calibration, validation, and final test data. Do not tune Q and R on the same sequence used to report final accuracy.

## 9. Measurements and performance metrics

Record synchronized timestamp, current, terminal voltage, temperature, reference SoC, estimated SoC, innovation, Kalman gain, Q, R, and fault status.

Report:

$$MAE=\frac{1}{N}\sum_{k=1}^{N}|z_{ref,k}-\hat z_k|$$

$$RMSE=\sqrt{\frac{1}{N}\sum_{k=1}^{N}(z_{ref,k}-\hat z_k)^2}$$

$$MAPE=\frac{100}{N}\sum_{k=1}^{N}\left|\frac{z_{ref,k}-\hat z_k}{z_{ref,k}}\right|$$

also report maximum absolute error, convergence time after initialization, computational time per sample, memory usage, covariance positive-semidefiniteness, and the number of SoC bound violations. The base paper reports MSE, MAE, MAPE, and R-squared and gives representative values of 0.0165%, 1.2%, 2%, and 99.34%, respectively; these should be treated as benchmark results from that paper, not as guaranteed outcomes for a different cell, dataset, or implementation [^1].

## 10. Controller and protection logic

The estimator output should feed a supervisory BMS layer:

- **Normal operation:** permit charge/discharge while SoC remains inside the configured operating window.
- **Low-SoC warning:** reduce load or request charging when the filtered SoC crosses the warning threshold.
- **Low-SoC protection:** disconnect the load if SoC remains below the protection threshold and voltage confirms the condition.
- **High-SoC protection:** stop charging when SoC and cell voltage exceed their limits.
- **Sensor fault:** fall back to a safe state when current, voltage, or temperature plausibility checks fail.
- **Estimator fault:** raise a diagnostic flag if innovation, covariance, or state bounds become abnormal.

Never use SoC estimation alone as the only protection mechanism. Hardware voltage, current, and temperature limits must remain independent safety barriers.

## 11. Hardware and software modules

**Hardware:** Li-ion/LiFePO4 pack, current sensor, precision voltage divider, temperature sensor, signal conditioning, ADC-capable microcontroller, load or electronic load, relay/MOSFET driver, isolated or appropriately protected communication interface, fuse, contactor, and emergency disconnect. The base paper demonstrates a 25.6-V, 12-Ah pack and a 60-MHz TMS320F28027 controller [^1].

**Firmware modules:** ADC acquisition; sensor calibration; digital filtering; timestamping; model parameter lookup; EKF prediction; EKF correction; SoC saturation; innovation monitoring; protection state machine; communications; watchdog; nonvolatile configuration.

**Offline tools:** MATLAB or Python for parameter identification, ACO tuning, plots, statistical comparison, and code-generation cross-checking.

## 12. Main risks and mitigations

- **Wrong OCV curve:** identify OCV at the selected chemistry and temperature; do not reuse the base-paper polynomial without validation.
- **Current-sensor bias:** calibrate zero-current offset and include bias checks; a small bias accumulates strongly in Coulomb counting.
- **Temperature variation:** log temperature and use temperature-indexed parameters or an adaptive model.
- **Battery ageing:** repeat identification at different state-of-health levels or add parameter adaptation. The base paper explicitly identifies temperature and SoH uncertainty as limitations [^1].
- **Overfitting Q and R:** use held-out validation profiles and repeated ACO runs.
- **EKF divergence:** use realistic initial covariance, Joseph update, covariance symmetry enforcement, innovation gating, and bounded states.
- **Poor excitation:** constant current alone may not identify both RC branches well; include pulses and rest/relaxation segments.
- **Embedded numerical failure:** use bounded values, finite checks, matrix-inversion safeguards, and a tested fallback estimator.
- **Unsafe testing:** use current limiting, fuses, thermal monitoring, protected cells, and an independent emergency disconnect.

## 13. Expected contribution

The project contribution should not be stated merely as “using ACO with EKF.” A stronger contribution is: **a reproducible, embedded, and experimentally validated method for selecting EKF covariance values, with explicit separation between model calibration, optimizer tuning, held-out validation, and real-time deployment.** The project will be stronger than a direct replication if it evaluates temperature, sensor bias, ageing, convergence, runtime, and robustness rather than reporting only one accuracy score.

## 14. Final project workflow

```text
Define cell and safety limits
        ↓
Calibrate current, voltage, and temperature sensors
        ↓
Collect OCV and pulse-response data
        ↓
Identify R–2RC parameters versus SoC and temperature
        ↓
Implement and verify Coulomb counting and EKF in simulation
        ↓
Run ACO to tune Q and R on calibration profiles
        ↓
Freeze Q/R and test on unseen profiles
        ↓
Port the verified estimator to the microcontroller
        ↓
Run hardware-in-the-loop tests
        ↓
Run controlled battery experiments
        ↓
Compare accuracy, robustness, runtime, and safety behavior
        ↓
Document limitations and reproducibility data
```

## 15. Recommended thesis/paper structure

1. Introduction and motivation
2. Literature review on SoC estimation and covariance tuning
3. Battery electrical modeling and parameter identification
4. EKF formulation
5. ACO-based Q/R optimization
6. Embedded system and controller architecture
7. Experimental design and reference-SoC method
8. Results: accuracy, convergence, runtime, and robustness
9. Discussion of temperature, ageing, and sensor uncertainty
10. Conclusion and future improvements

**Professional framing:** The estimator is the core technical contribution; the ACO tuner is the optimization contribution; the embedded implementation and independent test protocol are the validation contribution.

## 16. Complete conceptual interpretation

### What the project is—and is not

The project is an **online battery-state estimation system**. Its purpose is to infer an internal battery quantity that cannot be directly measured during normal operation. It is not a charger, inverter, motor controller, or energy-optimization algorithm. The ACO stage does not increase the battery’s physical charge and does not control the battery current; it selects estimator settings that reduce SoC estimation error.

The project has three separate layers:

1. **Physical layer:** the battery, current, voltage, and temperature sensors.
2. **Estimation layer:** the R–2RC model and EKF that infer SoC and hidden polarization states.
3. **Optimization and supervision layer:** ACO tunes Q/R offline, while BMS logic uses the estimated SoC for warnings and protection.

Confusing these layers is the most common conceptual error. The real SoC is a battery state; the estimated SoC is an algorithm output; Q and R are estimator design parameters.

### Why SoC is a state-space problem

The battery has internal variables that cannot be directly observed from one measurement. The terminal voltage is influenced simultaneously by OCV, current-dependent ohmic drop, transient polarization, temperature, and ageing. The R–2RC model makes these hidden effects explicit through three states: SoC, first-branch polarization voltage, and second-branch polarization voltage [^1].

A state-space model is useful because it separates:

- the **state transition equation**, which describes how the battery evolves;
- the **input**, which is current;
- the **measurement equation**, which predicts terminal voltage; and
- the **noise terms**, which represent model and sensor uncertainty.

The EKF then estimates the hidden state by repeatedly predicting and correcting it.

### Why a second-order RC model is selected

A simple resistance-only model cannot represent voltage relaxation after a current change. A one-RC model represents one transient time scale but may not capture both fast and slow polarization. The R–2RC model provides two transient branches and therefore a better practical compromise between fidelity, computational cost, and embedded implementation. The base paper selected an R–2RC network and used the three-state vector $$x=[SoC,V_p,V_c]^T$$ [^1].

This model is still an approximation. It does not automatically capture hysteresis, diffusion at all time scales, temperature gradients, cell-to-cell imbalance, or ageing. Those omissions are not reasons to reject the model; they define the model-error component that Q must represent and the conditions under which validation must be performed.

### Why EKF rather than only Coulomb counting

Coulomb counting is valuable as a baseline because it uses measured current and the capacity integral. However, its error accumulates over time and its result depends on the initial SoC and current-sensor bias. The EKF adds a correction pathway through terminal voltage and the battery model. When the model and voltage measurement are informative, the estimate can correct accumulated integration drift.

The EKF is chosen instead of a standard linear Kalman filter because both the battery model and the OCV relationship are nonlinear. It is chosen instead of a more computationally demanding nonlinear filter when a local linearization is adequate for the selected operating range. The base paper characterizes EKF as a nonlinear stochastic state estimator using prediction and correction steps [^1].

### Why ACO rather than manual tuning

The EKF does not know Q and R automatically. Manual tuning requires repeated experimentation: increase Q when the estimate is too rigid, increase R when voltage noise is being over-trusted, and repeat until the behavior is acceptable. This can work, but it is subjective and may not transfer across batteries or drive cycles.

ACO converts this tuning problem into a reproducible search. Each ant represents a candidate Q/R pair, each candidate runs the EKF, and the objective function scores the resulting SoC error. Better candidates attract the search, while random movement preserves exploration. The base paper formulates Q/R identification using SSE and bounds the search over Q and R [^1].

The key methodological requirement is to avoid overfitting. ACO must tune on calibration data, then Q and R must be frozen before testing on unseen current profiles. Otherwise, a low training error may simply mean that the optimizer memorized one experiment.

### What counts as a successful project

A successful project is not established by one low MAE value. It must demonstrate all of the following:

- the battery model predicts terminal voltage reasonably;
- the EKF remains numerically stable;
- the estimated SoC stays physically bounded;
- ACO improves or matches a defensible manually tuned baseline;
- the improvement persists on unseen current profiles;
- the estimator runs within the microcontroller sampling deadline;
- the result is robust to sensor noise and initial-SoC error;
- temperature and ageing limitations are explicitly measured; and
- independent hardware protection remains active if the estimator fails.

The paper reports promising accuracy and embedded feasibility, but it also states that temperature and SoH-related parameter uncertainty are not fully accounted for [^1]. Therefore, the strongest version of this project treats those effects as formal robustness experiments rather than ignoring them.

### Final research question

The project can be expressed as one precise research question:

> **Can an ACO-tuned EKF, based on a temperature-aware R–2RC battery model, estimate lithium-ion battery SoC more accurately and robustly than Coulomb counting and conventionally tuned EKF while satisfying real-time embedded-computation and safety requirements?**

### Final hypothesis

> **For the same battery, sensors, current profiles, and reference-SoC procedure, an EKF whose Q and R are selected by ACO will produce lower held-out SoC error, faster recovery from initialization error, and fewer estimator violations than a conventionally tuned EKF, without exceeding the embedded computation budget.**

### Minimum comparison matrix

The final experiments should compare at least:

1. Coulomb counting;
2. fixed manually tuned EKF;
3. ACO-tuned EKF;
4. ACO-tuned EKF with temperature-indexed parameters, if implemented; and
5. an ablation in which ACO is tuned on one profile but tested on another.

This comparison reveals whether the benefit comes from the EKF itself, from ACO tuning, or from inadvertent overfitting to a particular test sequence.


[^1]: Ant_Colony_Optimized_Extended_Kalman_Filter_for_State_of_Charge_Estimation_of_Lithium-Ion_Batteries.pdf.