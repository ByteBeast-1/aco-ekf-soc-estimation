/**
 * @file sil_main.c
 * @brief Software-in-the-loop driver: runs the firmware over measurements supplied by MATLAB.
 *
 * Usage:  sil_main <input.txt> <output.txt>
 *
 * Input file (plain text, numbers separated by white space):
 *   N                                   number of samples
 *   37 configuration numbers, in this order:
 *     R0_ref R1_ref R2_ref C1 C2 Qn eta dt Ea T_ref
 *     ocv[0..8]
 *     q_SOC q_V1 q_V2 r_V
 *     p0_SOC p0_V1 p0_V2
 *     init_soc
 *     v_min v_max t_min t_max i_max gate_nis gate_k0 max_rej nis_alpha nis_limit
 *   then N lines:  current_A  voltage_V  temperature_K
 * Output file: N lines:  soc_pct v_est innovation nis p_trace fault
 *
 * Numbers are parsed as double and rounded once to float, so the firmware sees exactly the float values
 * MATLAB printed with 9 significant digits.
 */

#include <stdio.h>
#include <stdlib.h>
#include "../bms_ekf.h"

static int read_f(FILE *f, float *dst)
{
    double v;
    if (fscanf(f, "%lf", &v) != 1) { return 0; }
    *dst = (float)v;
    return 1;
}

int main(int argc, char **argv)
{
    FILE *fin, *fout;
    BMS_BatteryParams_t p;
    BMS_EKF_Config_t c;
    BMS_Diag_Config_t d;
    BMS_EKF_t ekf;
    float init_soc, gate_k0, max_rej;
    long n, k;
    int i;

    if (argc < 3) {
        fprintf(stderr, "usage: %s input.txt output.txt\n", argv[0]);
        return 2;
    }
    fin = fopen(argv[1], "r");
    if (!fin) { fprintf(stderr, "cannot open %s\n", argv[1]); return 3; }
    fout = fopen(argv[2], "w");
    if (!fout) { fprintf(stderr, "cannot open %s\n", argv[2]); fclose(fin); return 3; }

    if (fscanf(fin, "%ld", &n) != 1 || n <= 0) { fprintf(stderr, "bad sample count\n"); return 4; }

    if (!(read_f(fin, &p.R0_ref) && read_f(fin, &p.R1_ref) && read_f(fin, &p.R2_ref) &&
          read_f(fin, &p.C1) && read_f(fin, &p.C2) && read_f(fin, &p.Qn) && read_f(fin, &p.eta) &&
          read_f(fin, &p.dt) && read_f(fin, &p.Ea) && read_f(fin, &p.T_ref))) { fprintf(stderr, "bad params\n"); return 4; }
    for (i = 0; i < BMS_OCV_COEFFS; i++) {
        if (!read_f(fin, &c.ocv[i])) { fprintf(stderr, "bad ocv\n"); return 4; }
    }
    if (!(read_f(fin, &c.q_SOC) && read_f(fin, &c.q_V1) && read_f(fin, &c.q_V2) && read_f(fin, &c.r_V) &&
          read_f(fin, &c.p0_SOC) && read_f(fin, &c.p0_V1) && read_f(fin, &c.p0_V2) &&
          read_f(fin, &init_soc))) { fprintf(stderr, "bad config\n"); return 4; }
    if (!(read_f(fin, &d.v_min) && read_f(fin, &d.v_max) && read_f(fin, &d.t_min) && read_f(fin, &d.t_max) &&
          read_f(fin, &d.i_max) && read_f(fin, &d.gate_nis) && read_f(fin, &gate_k0) && read_f(fin, &max_rej) &&
          read_f(fin, &d.nis_alpha) && read_f(fin, &d.nis_limit))) { fprintf(stderr, "bad diag\n"); return 4; }
    d.gate_k0 = (uint16_t)gate_k0;
    d.max_rej = (uint16_t)max_rej;

    BMS_EKF_Configure(&ekf, &p, &c, &d, init_soc);

    for (k = 0; k < n; k++) {
        float I, V, T;
        BMS_EKF_Out_t o;
        if (!(read_f(fin, &I) && read_f(fin, &V) && read_f(fin, &T))) { fprintf(stderr, "short input at %ld\n", k); return 5; }
        BMS_EKF_StepEx(&ekf, I, V, T, &o);
        fprintf(fout, "%.9g %.9g %.9g %.9g %.9g %u\n", (double)o.soc_pct, (double)o.v_est, (double)o.innovation,
                (double)o.nis, (double)o.p_trace, (unsigned int)o.fault);
    }
    fclose(fin);
    fclose(fout);
    return 0;
}
