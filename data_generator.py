"""
data_generator.py
=================
Generates realistic synthetic lithium-ion battery data for training and
testing the ACO-EKF SOC estimator.

Four operating point profiles are generated to match the paper's validation:
  1. Constant-current (CC) discharge at 0.5C
  2. Pulse discharge (periodic current steps)
  3. Dynamic / UDDS-like drive-cycle current
  4. Charging profile (CCCV-style)

Each profile includes:
  - Reference (ground-truth) SOC from the battery model
  - Measured terminal voltage with Gaussian sensor noise
  - Measured current with small Gaussian sensor noise

Usage
-----
    from data_generator import generate_all_profiles, BatteryDataset
    datasets = generate_all_profiles()   # returns list of 4 BatteryDataset
"""

import numpy as np
from dataclasses import dataclass
from battery_model import BatteryModel


# ---------------------------------------------------------------------------
# Dataset container
# ---------------------------------------------------------------------------
@dataclass
class BatteryDataset:
    """
    Container for one battery operating profile.

    Attributes
    ----------
    name         : str   – profile label
    time         : ndarray (N,)  – time axis [s]
    current_true : ndarray (N,)  – true current profile [A]
    current_meas : ndarray (N,)  – current with sensor noise [A]
    voltage_true : ndarray (N,)  – noise-free terminal voltage [V]
    voltage_meas : ndarray (N,)  – voltage with sensor noise [V]
    soc_ref      : ndarray (N,)  – reference SOC from model
    soc0         : float         – true initial SOC
    """
    name:         str
    time:         np.ndarray
    current_true: np.ndarray
    current_meas: np.ndarray
    voltage_true: np.ndarray
    voltage_meas: np.ndarray
    soc_ref:      np.ndarray
    soc0:         float


# ---------------------------------------------------------------------------
# Helper: add sensor noise
# ---------------------------------------------------------------------------
def _add_noise(
    signal:     np.ndarray,
    std:        float,
    rng:        np.random.Generator,
) -> np.ndarray:
    return signal + rng.normal(0.0, std, size=signal.shape)


# ---------------------------------------------------------------------------
# Profile 1: Constant-current discharge at 0.5C
# ---------------------------------------------------------------------------
def _profile_cc_discharge(
    model: BatteryModel,
    rng:   np.random.Generator,
    duration_s: int = 1800,
    c_rate: float   = 0.5,
    soc0:   float   = 0.95,
) -> BatteryDataset:
    """0.5C constant-current discharge from high SOC."""
    N   = duration_s
    I_C = model.Qn / 3600.0     # 1C current [A]
    I   = c_rate * I_C * np.ones(N)

    sim = model.simulate(I, soc0=soc0)
    vt  = sim["voltage"]

    return BatteryDataset(
        name         = "CC_Discharge_0.5C",
        time         = sim["time"],
        current_true = I,
        current_meas = _add_noise(I, std=0.005 * I_C, rng=rng),
        voltage_true = vt,
        voltage_meas = _add_noise(vt, std=0.005, rng=rng),
        soc_ref      = sim["soc"],
        soc0         = soc0,
    )


# ---------------------------------------------------------------------------
# Profile 2: Pulse discharge
# ---------------------------------------------------------------------------
def _profile_pulse(
    model: BatteryModel,
    rng:   np.random.Generator,
    soc0:  float = 0.90,
) -> BatteryDataset:
    """Repeated current pulses (30-second on / 30-second rest)."""
    I_C      = model.Qn / 3600.0
    pulse_on = 30
    pulse_off= 30
    n_pulses = 20
    I_mag    = 1.0 * I_C

    cycle = np.concatenate([
        I_mag * np.ones(pulse_on),
        np.zeros(pulse_off),
    ])
    I = np.tile(cycle, n_pulses)
    N = len(I)

    sim = model.simulate(I, soc0=soc0)
    vt  = sim["voltage"]

    return BatteryDataset(
        name         = "Pulse_Discharge_1C",
        time         = sim["time"],
        current_true = I,
        current_meas = _add_noise(I, std=0.008 * I_C, rng=rng),
        voltage_true = vt,
        voltage_meas = _add_noise(vt, std=0.005, rng=rng),
        soc_ref      = sim["soc"],
        soc0         = soc0,
    )


# ---------------------------------------------------------------------------
# Profile 3: Dynamic / UDDS-like drive cycle
# ---------------------------------------------------------------------------
def _profile_dynamic(
    model: BatteryModel,
    rng:   np.random.Generator,
    duration_s: int = 1800,
    soc0:       float = 0.85,
) -> BatteryDataset:
    """
    Stochastic drive-cycle current profile with varying amplitude and rests.
    Approximates a UDDS-style test (urban dynamometer driving schedule).
    """
    I_C = model.Qn / 3600.0
    N   = duration_s

    # Build a piece-wise random current profile
    I = np.zeros(N)
    k = 0
    while k < N:
        # Choose segment type: heavy discharge, light discharge, or rest
        seg_type = rng.choice(["heavy", "light", "rest"], p=[0.30, 0.45, 0.25])
        seg_len  = int(rng.integers(15, 90))
        seg_end  = min(k + seg_len, N)

        if seg_type == "heavy":
            amplitude = rng.uniform(0.8, 1.5) * I_C
        elif seg_type == "light":
            amplitude = rng.uniform(0.2, 0.7) * I_C
        else:
            amplitude = 0.0

        I[k:seg_end] = amplitude
        k = seg_end

    sim = model.simulate(I, soc0=soc0)
    vt  = sim["voltage"]

    return BatteryDataset(
        name         = "Dynamic_Drive_Cycle",
        time         = sim["time"],
        current_true = I,
        current_meas = _add_noise(I, std=0.01 * I_C, rng=rng),
        voltage_true = vt,
        voltage_meas = _add_noise(vt, std=0.005, rng=rng),
        soc_ref      = sim["soc"],
        soc0         = soc0,
    )


# ---------------------------------------------------------------------------
# Profile 4: CCCV charge profile
# ---------------------------------------------------------------------------
def _profile_cccv_charge(
    model:  BatteryModel,
    rng:    np.random.Generator,
    soc0:   float = 0.20,
    cc_rate: float = 0.5,
    cv_voltage: float = 4.18,
) -> BatteryDataset:
    """
    Constant-current phase followed by constant-voltage taper.
    Models a standard charger profile.
    """
    I_C  = model.Qn / 3600.0
    I_cc = cc_rate * I_C    # discharge-sign negative for charging
    dt   = model.dt

    currents  = []
    soc_list  = []
    vt_list   = []

    soc = soc0
    vp  = 0.0
    x   = np.array([soc, vp])

    # ── CC phase: charge until terminal voltage hits cv_voltage ──────
    for _ in range(30000):
        I_now = -I_cc    # negative = charging
        vt    = model.terminal_voltage(x, I_now)

        currents.append(I_now)
        soc_list.append(x[0])
        vt_list.append(vt)

        if vt >= cv_voltage or x[0] >= 0.99:
            break
        x = model.state_transition(x, I_now)

    # ── CV phase: exponentially tapering current ──────────────────────
    I_cv   = -I_cc
    taper  = np.exp(-np.arange(1800) / 300.0)
    for t in range(1800):
        I_now = I_cv * taper[t]
        vt    = model.terminal_voltage(x, I_now)

        currents.append(I_now)
        soc_list.append(x[0])
        vt_list.append(vt)

        if abs(I_now) < 0.02 * I_C or x[0] >= 0.995:
            break
        x = model.state_transition(x, I_now)

    currents = np.array(currents)
    soc_arr  = np.array(soc_list)
    vt_arr   = np.array(vt_list)
    N        = len(currents)

    return BatteryDataset(
        name         = "CCCV_Charge_0.5C",
        time         = np.arange(N, dtype=float) * dt,
        current_true = currents,
        current_meas = _add_noise(currents, std=0.005 * I_C, rng=rng),
        voltage_true = vt_arr,
        voltage_meas = _add_noise(vt_arr, std=0.005, rng=rng),
        soc_ref      = soc_arr,
        soc0         = soc0,
    )


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------
def generate_all_profiles(
    model: BatteryModel | None = None,
    seed:  int                 = 0,
) -> list[BatteryDataset]:
    """
    Generate all four operating-point profiles.

    Parameters
    ----------
    model : BatteryModel, optional
        Battery model to use.  Default parameters used if not supplied.
    seed  : int
        Random seed for sensor-noise generation.

    Returns
    -------
    list of 4 BatteryDataset objects:
        [CC_discharge, Pulse, Dynamic, CCCV_charge]
    """
    if model is None:
        model = BatteryModel()

    rng = np.random.default_rng(seed)

    datasets = [
        _profile_cc_discharge(model, rng),
        _profile_pulse(model, rng),
        _profile_dynamic(model, rng),
        _profile_cccv_charge(model, rng),
    ]

    print(f"  [Data] Generated {len(datasets)} operating profiles:")
    for ds in datasets:
        duration_min = ds.time[-1] / 60.0
        soc_range    = f"{ds.soc_ref.min()*100:.1f}% → {ds.soc_ref.max()*100:.1f}%"
        print(f"         {ds.name:<28}  {len(ds.time):5d} samples  "
              f"{duration_min:6.1f} min  SOC {soc_range}")

    return datasets
