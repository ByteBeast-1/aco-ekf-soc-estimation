# c_firmware

ANSI C implementation of the 3-state 2-RC thermal EKF with diagnostics. It is the C twin of
`simulink_matlab/lib/bms_est_step.m` (estimator mode 0): same equations, same order of operations,
same limits. Only the arithmetic differs (32-bit float).

| File | Role |
| --- | --- |
| `bms_ekf.h`, `bms_ekf.c` | The estimator. No dynamic memory; one `BMS_EKF_t` object holds all state |
| `main_test.c` | Console smoke test: 10 samples, prints SOC, NIS and the diagnostic code |
| `sil/sil_main.c` | Software-in-the-loop driver: reads measurements from a text file, writes the estimates |
| `build/` | Compiled programs (generated, git-ignored) |

## Interface

```c
BMS_EKF_Init(&ekf, init_soc, temp_k);                       /* default cell, ACO-tuned noise, diagnostics */
BMS_EKF_Configure(&ekf, &params, &config, &diag, init_soc); /* your own cell, OCV curve, noise, limits   */
BMS_EKF_StepEx(&ekf, current_A, voltage_V, temp_k, &out);   /* once per sample; out has SOC %, NIS, fault */
```

`out.fault` is a bit mask: 1 voltage range, 2 temperature range, 4 current range, 8 innovation rejected,
16 filter inconsistent. Memory: `sizeof(BMS_EKF_t)` is printed by `main_test` (236 bytes: the fields add up to 235 and the compiler pads to a
multiple of 4 for float alignment; this includes the cell parameters, OCV curve and diagnostic limits).

## Build and run

The easy way is from MATLAB; it uses `gcc` if present and otherwise the LCC compiler bundled with MATLAB:

```matlab
cd simulink_matlab
exe = bms_sil_build('test');  system(['"' exe '"'])     % smoke test
run_bms_regression                                      % suite 7 = SIL against the MATLAB estimator
```

With gcc directly (not exercised on this machine, which has no gcc):

```bash
gcc -std=c99 -O2 -Wall -Wextra -o build/bms_test bms_ekf.c main_test.c -lm
gcc -std=c99 -O2 -Wall -Wextra -o build/bms_sil  bms_ekf.c sil/sil_main.c -lm
```

## What the SIL test proves

`simulink_matlab/lib/bms_sil_run.m` runs the MATLAB estimator in `single` precision and the compiled firmware on
exactly the same float measurements (simulated cell with voltage glitches and an open thermistor, and real NASA
discharge and charge cycles) and compares SOC, predicted voltage, covariance trace and the diagnostic codes.
The agreement limits are in `run_bms_regression.m` (suite 7).

## Known limits

* Estimator mode 0 only: no adaptive noise, no R0 tracking, no capacity state (those exist only in MATLAB).
* The innovation gate arms after 30 samples, so a glitch in the first 30 s is not rejected (see `main_test.c`).
* Verified only with the LCC-win64 compiler. Static analysis (MISRA), fixed-point, stack and execution-time
  measurement on a real target are still open.
