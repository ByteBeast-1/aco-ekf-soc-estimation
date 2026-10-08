/**
 * @file main_test.c
 * @brief C Firmware Test Harness for 3-State 2-RC Thermal EKF.
 */

#include <stdio.h>
#include <stdlib.h>
#include "bms_ekf.h"

int main(void) {
    printf("=================================================================\n");
    printf("  ANSI C Firmware Test: 3-State 2-RC Thermal EKF SOC Estimator  \n");
    printf("=================================================================\n\n");

    BMS_EKF_t bms_ekf;
    float init_soc_guess = 0.85f; // Initial guess with 10% error (True SOC = 0.95)
    float temp_k = 298.15f;       // 25°C

    BMS_EKF_Init(&bms_ekf, init_soc_guess, temp_k);

    float current_profile[10] = { 2.5f, 2.5f, 2.5f, 2.5f, 2.5f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
    float voltage_meas[10]    = { 3.95f, 3.92f, 3.90f, 3.88f, 3.86f, 3.91f, 3.93f, 3.94f, 3.95f, 3.96f };

    printf("Step | Current (A) | Meas Volt (V) | Est SOC (%%) | Est V1 (mV) | Est V2 (mV)\n");
    printf("-------------------------------------------------------------------------\n");

    for (int k = 0; k < 10; k++) {
        float est_soc = 0.0f;
        float est_v1  = 0.0f;
        float est_v2  = 0.0f;

        BMS_EKF_Step(&bms_ekf, current_profile[k], voltage_meas[k], temp_k, &est_soc, &est_v1, &est_v2);

        printf(" %2d  |   %5.2f     |    %6.3f     |   %6.2f%%   |   %6.2f    |   %6.2f   \n",
               k + 1, current_profile[k], voltage_meas[k], est_soc * 100.0f, est_v1 * 1000.0f, est_v2 * 1000.0f);
    }

    printf("-------------------------------------------------------------------------\n");
    printf("  [Firmware Test] 3-State 2-RC EKF compiled and executed cleanly!\n");
    printf("=================================================================\n");

    return 0;
}
