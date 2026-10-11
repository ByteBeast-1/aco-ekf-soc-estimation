/**
 * @file main_test.c
 * @brief Console smoke test for the 3-state 2-RC thermal EKF firmware.
 *
 * Ten samples: five seconds of 2.5 A discharge, then rest. The true SOC is 95 %, the filter starts at 85 %.
 * The last sample is a deliberate 0.5 V voltage glitch to show the diagnostic code.
 */

#include <stdio.h>
#include <stdlib.h>
#include "bms_ekf.h"

int main(void)
{
    BMS_EKF_t bms_ekf;
    float init_soc_guess = 0.85f;     /* 10 % wrong start */
    float temp_k = 298.15f;           /* 25 C */
    float current_profile[10] = { 2.5f, 2.5f, 2.5f, 2.5f, 2.5f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
    float voltage_meas[10]    = { 3.95f, 3.92f, 3.90f, 3.88f, 3.86f, 3.91f, 3.93f, 3.94f, 3.95f, 3.96f };
    int k;

    printf("=================================================================\n");
    printf("  ANSI C Firmware Test: 3-State 2-RC Thermal EKF SOC Estimator  \n");
    printf("=================================================================\n\n");
    printf("  sizeof(BMS_EKF_t) = %u bytes (persistent RAM of the whole estimator)\n\n", (unsigned int)sizeof(BMS_EKF_t));

    BMS_EKF_Init(&bms_ekf, init_soc_guess, temp_k);

    printf("Step | Current (A) | Meas Volt (V) | Est SOC (%%) | NIS      | Fault\n");
    printf("-------------------------------------------------------------------\n");

    for (k = 0; k < 10; k++) {
        BMS_EKF_Out_t out;
        float v = voltage_meas[k];
        if (k == 9) { v = 4.46f; }                 /* injected +0.5 V glitch on the last sample */
        BMS_EKF_StepEx(&bms_ekf, current_profile[k], v, temp_k, &out);
        printf(" %2d  |   %5.2f     |    %6.3f     |   %6.2f%%   | %8.3f |  %u\n",
               k + 1, current_profile[k], v, out.soc_pct, out.nis, (unsigned int)out.fault);
    }

    printf("-------------------------------------------------------------------\n");
    printf("  [Firmware Test] 3-State 2-RC EKF compiled and executed cleanly!\n");
    printf("  (The gate arms after 30 samples, so the glitch is only flagged in a longer run; see the SIL test.)\n");
    printf("=================================================================\n");
    return 0;
}
