"""
data_generator.py
=================
Generates realistic synthetic lithium-ion battery data for training and
testing the 2-RC Thermal-Adaptive MO-ACO-EKF SOC estimator.

Four operating point profiles are generated:
  1. Constant-current (CC) discharge at 0.5C
  2. Pulse discharge (periodic current steps)
  3. Dynamic / UDDS-like drive-cycle current
  4. Charging profile (CCCV-style)

Includes temperature transients (cell self-heating 25°C -> 39°C).
"""

import numpy as np
from dataclasses import dataclass
from battery_model import BatteryModel


@dataclass
class BatteryDataset:
    """
    Container for one battery operating profile.
    """
    name:         str
    time:         np.ndarray
    current_true: np.ndarray
    current_meas: np.ndarray
    voltage_true: np.ndarray
    voltage_meas: np.ndarray
    soc_ref:      np.ndarray
    temp_k:       np.ndarray
    soc0:         float


def _add_noise(signal: np.ndarray, std: float, rng: np.random.Generator) -> np.ndarray:
    return signal + rng.normal(0.0, std, size=signal.shape)


def _profile_cc_discharge(model: BatteryModel, rng: np.random.Generator) -> BatteryDataset:
    dt = model.dt
    t_end = 1800.0  # 30 mins
    N = int(t_end / dt) + 1
    time_ax = np.linspace(0.0, t_end, N)

    current_true = np.full(N, 1.0)  # 1.0 A discharge (0.5C)
    current_meas = _add_noise(current_true, 0.02, rng)

    soc0 = 0.95
    x = np.array([soc0, 0.0, 0.0])
    soc_ref = np.zeros(N)
    v_true  = np.zeros(N)
    temp_k  = np.linspace(298.15, 308.15, N)  # Temperature heating

    for k in range(N):
        soc_ref[k] = x[0]
        v_true[k]  = model.terminal_voltage(x, current_true[k], temp_k[k])
        x = model.step(x, current_true[k], temp_k[k])

    v_meas = _add_noise(v_true, 0.008, rng)

    return BatteryDataset(
        name="CC_Discharge_0.5C",
        time=time_ax,
        current_true=current_true,
        current_meas=current_meas,
        voltage_true=v_true,
        voltage_meas=v_meas,
        soc_ref=soc_ref,
        temp_k=temp_k,
        soc0=soc0
    )


def _profile_pulse(model: BatteryModel, rng: np.random.Generator) -> BatteryDataset:
    dt = model.dt
    t_end = 1200.0  # 20 mins
    N = int(t_end / dt) + 1
    time_ax = np.linspace(0.0, t_end, N)

    current_true = np.zeros(N)
    for k in range(N):
        cycle = (time_ax[k] % 90.0)
        current_true[k] = 2.0 if cycle < 60.0 else 0.0

    current_meas = _add_noise(current_true, 0.02, rng)

    soc0 = 0.90
    x = np.array([soc0, 0.0, 0.0])
    soc_ref = np.zeros(N)
    v_true  = np.zeros(N)
    temp_k  = np.linspace(298.15, 305.15, N)

    for k in range(N):
        soc_ref[k] = x[0]
        v_true[k]  = model.terminal_voltage(x, current_true[k], temp_k[k])
        x = model.step(x, current_true[k], temp_k[k])

    v_meas = _add_noise(v_true, 0.008, rng)

    return BatteryDataset(
        name="Pulse_Discharge_1C",
        time=time_ax,
        current_true=current_true,
        current_meas=current_meas,
        voltage_true=v_true,
        voltage_meas=v_meas,
        soc_ref=soc_ref,
        temp_k=temp_k,
        soc0=soc0
    )


def _profile_dynamic(model: BatteryModel, rng: np.random.Generator) -> BatteryDataset:
    dt = model.dt
    t_end = 1800.0  # 30 mins
    N = int(t_end / dt) + 1
    time_ax = np.linspace(0.0, t_end, N)

    base = 1.0 + 0.8 * np.sin(2 * np.pi * time_ax / 200.0)
    pulses = 1.2 * (np.sin(2 * np.pi * time_ax / 35.0) > 0.3)
    current_true = np.clip(base + pulses, 0.0, 3.5)
    current_meas = _add_noise(current_true, 0.025, rng)

    soc0 = 0.85
    x = np.array([soc0, 0.0, 0.0])
    soc_ref = np.zeros(N)
    v_true  = np.zeros(N)
    temp_k  = np.linspace(298.15, 312.15, N)

    for k in range(N):
        soc_ref[k] = x[0]
        v_true[k]  = model.terminal_voltage(x, current_true[k], temp_k[k])
        x = model.step(x, current_true[k], temp_k[k])

    v_meas = _add_noise(v_true, 0.01, rng)

    return BatteryDataset(
        name="Dynamic_Drive_Cycle",
        time=time_ax,
        current_true=current_true,
        current_meas=current_meas,
        voltage_true=v_true,
        voltage_meas=v_meas,
        soc_ref=soc_ref,
        temp_k=temp_k,
        soc0=soc0
    )


def _profile_cccv_charge(model: BatteryModel, rng: np.random.Generator) -> BatteryDataset:
    dt = model.dt
    soc0 = 0.20
    x = np.array([soc0, 0.0, 0.0])

    t_list, i_list, v_list, s_list, T_list = [], [], [], [], []
    t = 0.0
    v_max = 4.15
    i_charge = -1.0  # Negative = charging

    # Stage 1: Constant Current
    while x[0] < 0.80:
        vt = model.terminal_voltage(x, i_charge, 298.15)
        t_list.append(t)
        i_list.append(i_charge)
        v_list.append(vt)
        s_list.append(x[0])
        T_list.append(298.15)
        x = model.step(x, i_charge, 298.15)
        t += dt

    # Stage 2: Constant Voltage taper
    i_cv = i_charge
    while i_cv < -0.05 and t < 5200.0:
        vt = model.terminal_voltage(x, i_cv, 298.15)
        t_list.append(t)
        i_list.append(i_cv)
        v_list.append(vt)
        s_list.append(x[0])
        T_list.append(298.15)
        x = model.step(x, i_cv, 298.15)
        i_cv = min(i_cv * 0.9985, -0.05)
        t += dt

    time_ax      = np.array(t_list)
    current_true = np.array(i_list)
    v_true       = np.array(v_list)
    soc_ref      = np.array(s_list)
    temp_k       = np.array(T_list)

    current_meas = _add_noise(current_true, 0.015, rng)
    v_meas       = _add_noise(v_true, 0.006, rng)

    return BatteryDataset(
        name="CCCV_Charge_0.5C",
        time=time_ax,
        current_true=current_true,
        current_meas=current_meas,
        voltage_true=v_true,
        voltage_meas=v_meas,
        soc_ref=soc_ref,
        temp_k=temp_k,
        soc0=soc0
    )


def generate_all_profiles(model: BatteryModel | None = None, seed: int = 42) -> list[BatteryDataset]:
    """Generate all 4 synthetic operating point profiles."""
    if model is None:
        model = BatteryModel()

    rng = np.random.default_rng(seed)

    datasets = [
        _profile_cc_discharge(model, rng),
        _profile_pulse(model, rng),
        _profile_dynamic(model, rng),
        _profile_cccv_charge(model, rng),
    ]

    print(f"  [Data] Generated {len(datasets)} 2-RC operating profiles:")
    for ds in datasets:
        duration_min = ds.time[-1] / 60.0
        soc_range    = f"{ds.soc_ref.min()*100:.1f}% -> {ds.soc_ref.max()*100:.1f}%"
        print(f"         {ds.name:<28}  {len(ds.time):5d} samples  "
              f"{duration_min:6.1f} min  SOC {soc_range}")

    return datasets
