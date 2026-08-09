/**
 * @file main_test.c
 * @brief Standalone test runner for embedded BMS EKF ANSI C implementation.
 */

#include <stdio.h>
#include "bms_ekf.h"

int main(void) {
    printf("=================================================================\n");
    printf("  Automotive BMS Embedded C EKF Firmware Module Test\n");
    printf("=================================================================\n\n");

    /* 1. Set Battery Model Parameters */
    BMS_BatteryParams_t params = {
        .R0  = 0.015f,
        .R1  = 0.010f,
        .C1  = 3000.0f,
        .Qn  = 7200.0f, /* 2 Ah = 7200 As */
        .eta = 1.0f,
        .dt  = 1.0f
    };

    /* 2. Set ACO-Tuned Covariances */
    BMS_EKF_Config_t config = {
        .q_SOC = 1.0e-7f,
        .q_Vp  = 1.0e-7f,
        .r_V   = 2.455e-3f
    };

    /* 3. Initialize EKF Handle with +10% Initial SOC Error */
    BMS_EKF_t bms_ekf;
    float initial_soc_guess = 0.85f; /* True starting SOC is 0.75f */
    BMS_EKF_Init(&bms_ekf, &params, &config, initial_soc_guess);

    printf("  [Init] Initialized EKF with Initial SOC Guess = %.2f%%\n", initial_soc_guess * 100.0f);
    printf("  [Init] Configured ACO Q = diag([%.1e, %.1e]), R = %.3e\n\n", config.q_SOC, config.q_Vp, config.r_V);

    /* 4. Run 10 Simulated Steps of Constant Current Discharge (1A) */
    printf("  Step | Discharge Current | Measured Voltage | Estimated SOC\n");
    printf("  -----------------------------------------------------------\n");

    float current_A = 1.0f; /* 1A discharge */
    float mock_voltages[10] = { 3.82f, 3.81f, 3.80f, 3.79f, 3.78f, 3.77f, 3.76f, 3.75f, 3.74f, 3.73f };

    for (int step = 1; step <= 10; step++) {
        float soc_est = BMS_EKF_Update(&bms_ekf, current_A, mock_voltages[step - 1]);
        printf("  %4d | %15.2f A | %14.2f V | %11.4f %%\n", step, current_A, mock_voltages[step - 1], soc_est * 100.0f);
    }

    printf("  -----------------------------------------------------------\n");
    printf("  [Pass] Embedded C EKF execution completed successfully.\n");
    return 0;
}
