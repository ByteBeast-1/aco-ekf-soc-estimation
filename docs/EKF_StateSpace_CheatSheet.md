# ⚡ Ultimate Cheat Sheet: EKF & State-Space Mathematics

---

## 1. 📐 Overall State-Space Model (2-RC Thermal ECM)

### State Vector & Input
$$\mathbf{x}_k = \begin{bmatrix} \text{SOC}_k \\ V_{1,k} \\ V_{2,k} \end{bmatrix}_{3 \times 1}, \qquad u_k = I_k \quad (\text{Current in Amperes})$$

---

### Arrhenius Thermal Parameter Equations
$$t_{\text{factor}}(T_k) = \exp\left( \frac{E_a}{R_{\text{gas}}} \left( \frac{1}{T_{k}} - \frac{1}{T_{\text{ref}}} \right) \right)$$

$$R_0(T_k) = R_{0,\text{base}} \cdot t_{\text{factor}}(T_k), \quad R_1(T_k) = R_{1,\text{base}} \cdot t_{\text{factor}}(T_k), \quad R_2(T_k) = R_{2,\text{base}} \cdot t_{\text{factor}}(T_k)$$

*(Constants: $E_a = 18000\text{ J/mol}$, $R_{\text{gas}} = 8.314\text{ J/(mol}\cdot\text{K)}$, $T_{\text{ref}} = 298.15\text{ K}$)*

---

### Discrete-Time State Transition Equation
$$\mathbf{x}_{k+1} = \mathbf{F}_k \mathbf{x}_k + \mathbf{B}_k u_k$$

$$\begin{bmatrix} \text{SOC}_{k+1} \\ V_{1,k+1} \\ V_{2,k+1} \end{bmatrix} = \begin{bmatrix} 1 & 0 & 0 \\ 0 & e^{-\frac{\Delta t}{R_1(T) C_1}} & 0 \\ 0 & 0 & e^{-\frac{\Delta t}{R_2(T) C_2}} \end{bmatrix} \begin{bmatrix} \text{SOC}_k \\ V_{1,k} \\ V_{2,k} \end{bmatrix} + \begin{bmatrix} -\frac{\eta \Delta t}{Q_n} \\ R_1(T)\left(1 - e^{-\frac{\Delta t}{R_1(T) C_1}}\right) \\ R_2(T)\left(1 - e^{-\frac{\Delta t}{R_2(T) C_2}}\right) \end{bmatrix} I_k$$

---

### Measurement (Terminal Voltage) Output Equation
$$V_{t,k} = h(\mathbf{x}_k, u_k, T_k) = \text{OCV}(\text{SOC}_k) - V_{1,k} - V_{2,k} - I_k \cdot R_0(T_k)$$

---

### Measurement Jacobian Matrix ($\mathbf{H}_k$)
$$\mathbf{H}_k = \left. \frac{\partial h}{\partial \mathbf{x}} \right|_{\hat{\mathbf{x}}_{k|k-1}} = \begin{bmatrix} \frac{\partial \text{OCV}}{\partial \text{SOC}} & -1.0 & -1.0 \end{bmatrix}_{1 \times 3}$$

---

## 2. 🧠 Extended Kalman Filter (EKF) Complete Formulae

```text
========================================================================================
                                INITIALIZATION (t = 0)
========================================================================================
Initial State:          x_hat_0 = [0.85, 0.0, 0.0]^T   (10% Intentional Test Error)
Initial Covariance:     P_0     = diag([0.01, 0.0025, 0.0025])
Process Noise Matrix:   Q*      = diag([1e-7, 1e-7, 1e-7])       (from MO-ACO)
Sensor Noise Matrix:    R*      = 2.455e-3                       (from MO-ACO)
========================================================================================
```

---

### 🔁 The Recursive 6-Step Loop (Runs Every Second $k = 1, 2, 3 \dots$)

$$\begin{array}{rll}
\hline
\textbf{Step} & \textbf{Name} & \textbf{Exact Mathematical Formula} \\
\hline
\\[-6pt]
\textbf{1.} & \textbf{State Prediction} & \hat{\mathbf{x}}_{k|k-1} = \mathbf{F}_k \hat{\mathbf{x}}_{k-1|k-1} + \mathbf{B}_k u_k \\[8pt]
\textbf{2.} & \textbf{Covariance Prediction} & \mathbf{P}_{k|k-1} = \mathbf{F}_k \mathbf{P}_{k-1|k-1} \mathbf{F}_k^T + \mathbf{Q}^* \\[8pt]
\textbf{3.} & \textbf{Voltage Residual} & r_k = V_{\text{meas},k} - \Big( \text{OCV}(\hat{\text{SOC}}_{k|k-1}) - \hat{V}_{1,k|k-1} - \hat{V}_{2,k|k-1} - I_k R_0(T_k) \Big) \\[8pt]
\textbf{4.} & \textbf{Innovation Covariance} & S_k = \mathbf{H}_k \mathbf{P}_{k|k-1} \mathbf{H}_k^T + R^* \\[8pt]
\textbf{5.} & \textbf{Kalman Gain} & \mathbf{K}_k = \mathbf{P}_{k|k-1} \mathbf{H}_k^T S_k^{-1} \\[8pt]
\textbf{6.} & \textbf{State Correction} & \hat{\mathbf{x}}_{k|k} = \hat{\mathbf{x}}_{k|k-1} + \mathbf{K}_k \cdot r_k \\[8pt]
\textbf{7.} & \textbf{Joseph-Form Covariance} & \mathbf{P}_{k|k} = (\mathbf{I} - \mathbf{K}_k \mathbf{H}_k) \mathbf{P}_{k|k-1} (\mathbf{I} - \mathbf{K}_k \mathbf{H}_k)^T + \mathbf{K}_k R^* \mathbf{K}_k^T \\[6pt]
\hline
\end{array}$$

---

## 3. 🛡️ Why Joseph-Form? (Formula Quick Reference)

$$\text{Standard Form (Crashes on 32-bit ECUs):} \quad \mathbf{P} = (\mathbf{I} - \mathbf{K}\mathbf{H})\mathbf{P}_{pred}$$

$$\text{Joseph Form (Guaranteed Positive-Definite):} \quad \mathbf{P} = \underbrace{(\mathbf{I} - \mathbf{K}\mathbf{H})\mathbf{P}_{pred}(\mathbf{I} - \mathbf{K}\mathbf{H})^T}_{\mathbf{A} \cdot \mathbf{P} \cdot \mathbf{A}^T \ge 0} + \underbrace{\mathbf{K} R^* \mathbf{K}^T}_{\mathbf{B} \cdot R \cdot \mathbf{B}^T > 0}$$

---

## 4. 🐜 MO-ACO Fitness Function

$$\text{Fitness}(Q, R) = w_1 \cdot \text{RMSE}(\text{SOC}) + w_2 \cdot \text{Trace}(\mathbf{P})$$

* **$\text{RMSE}(\text{SOC})$:** $\sqrt{\frac{1}{N} \sum_{k=1}^N (\text{SOC}_{\text{true},k} - \hat{\text{SOC}}_k)^2}$ $\rightarrow$ **Accuracy**
* **$\text{Trace}(\mathbf{P})$:** $P_{11} + P_{22} + P_{33}$ $\rightarrow$ **Mathematical Stability**
