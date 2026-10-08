"""
parse_official_nasa_mat.py
==========================
Parses the official MATLAB .mat dataset archive (data_raw/B0005.mat) from the
NASA Ames Prognostics Data Repository.

Extracts real experimental 18650 Li-ion cell measurements:
  - Time [s]
  - Measured Current [A]
  - Measured Voltage [V]
  - Measured Temperature [°C]
  - Reference SOC calculated via Coulomb Integration on NASA capacity
"""

import os
import scipy.io
import numpy as np

def parse_nasa_b0005_mat(cycle_num: int = 1) -> str:
    mat_path = os.path.join("data_raw", "B0005.mat")
    if not os.path.exists(mat_path):
        raise FileNotFoundError(f"Missing '{mat_path}'. Run download_official_nasa_mat.py first.")

    mat = scipy.io.loadmat(mat_path)
    struct = mat["B0005"][0, 0]
    cycles = struct["cycle"][0]

    discharge_count = 0
    target_data = None

    for c in cycles:
        if c["type"][0] == "discharge":
            discharge_count += 1
            if discharge_count == cycle_num:
                target_data = c["data"][0, 0]
                break

    if target_data is None:
        raise ValueError(f"Discharge cycle {cycle_num} not found in B0005.mat")

    raw_t = target_data["Time"][0].flatten()
    raw_v = target_data["Voltage_measured"][0].flatten()
    raw_i = np.abs(target_data["Current_measured"][0].flatten())
    raw_temp = target_data["Temperature_measured"][0].flatten()
    capacity = float(target_data["Capacity"][0, 0]) if "Capacity" in target_data.dtype.names else 1.856

    min_len   = min(len(raw_t), len(raw_v), len(raw_i), len(raw_temp))
    time_s    = raw_t[:min_len]
    voltage_v = raw_v[:min_len]
    current_a = raw_i[:min_len]
    temp_c    = raw_temp[:min_len]

    cum_amp_sec = np.cumsum(current_a * np.gradient(time_s))
    soc_ref = 1.0 - (cum_amp_sec / (capacity * 3600.0))
    soc_ref = np.clip(soc_ref, 0.0, 1.0)

    os.makedirs("data", exist_ok=True)
    csv_path = os.path.join("data", f"NASA_B0005_OFFICIAL_Cycle{cycle_num}.csv")

    np.savetxt(
        csv_path,
        np.column_stack([time_s, current_a, voltage_v, temp_c, soc_ref]),
        header="time_s,current_A,voltage_V,temperature_C,soc_ref",
        delimiter=",",
        comments="",
    )

    print(f"  [Official NASA Parser] Extracted Cycle {cycle_num} ({len(time_s)} samples, Capacity={capacity:.3f} Ah)")
    print(f"                         Saved to '{csv_path}'")
    return csv_path

if __name__ == "__main__":
    parse_nasa_b0005_mat(cycle_num=1)
