/**
 * @file bms_ekf.h
 * @brief Production ANSI C 3-State 2-RC Thermal EKF for Automotive BMS.
 * @author BMS Firmware Engineering Team
 *
 * Target: ARM Cortex-M0+/M3/M4/M7, STM32, TI C2000 / TMS320F28027
 * RAM Usage: < 2.0 KB (zero dynamic memory allocation - no malloc)
 */

#ifndef BMS_EKF_H
#define BMS_EKF_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <math.h>

#define BMS_STATE_DIM       3       /* x = [SOC, V1, V2]^T */
#define BMS_MEAS_DIM        1       /* y = [Vt]            */

typedef struct {
    float R0_ref;   /* Ohmic resistance at 25C [Ohm]               */
    float R1_ref;   /* Fast polarization resistance at 25C [Ohm]    */
    float C1;       /* Fast polarization capacitance [Farad]       */
    float R2_ref;   /* Slow polarization resistance at 25C [Ohm]    */
    float C2;       /* Slow polarization capacitance [Farad]       */
    float Qn;       /* Nominal Capacity [Ampere-seconds] (As)       */
    float eta;      /* Coulombic efficiency                         */
    float dt;       /* Sampling period [seconds]                    */
    float Ea;       /* Activation energy [J/mol]                    */
    float T_ref;    /* Reference temperature [K] (298.15 K)         */
} BMS_BatteryParams_t;

typedef struct {
    float q_SOC;    /* Process noise variance for SOC state         */
    float q_V1;     /* Process noise variance for V1 state          */
    float q_V2;     /* Process noise variance for V2 state          */
    float r_V;      /* Measurement noise variance for Voltage       */
} BMS_EKF_Config_t;

typedef struct {
    BMS_BatteryParams_t params;
    BMS_EKF_Config_t    config;

    float x_hat[BMS_STATE_DIM];                 /* State vector [SOC, V1, V2] */
    float P[BMS_STATE_DIM][BMS_STATE_DIM];      /* Covariance matrix P        */
    
    uint8_t is_initialized;
} BMS_EKF_t;

void BMS_EKF_Init(BMS_EKF_t *ekf, float init_soc, float temp_k);
void BMS_EKF_Step(BMS_EKF_t *ekf, float current_A, float voltage_meas_V, float temp_k, float *out_soc, float *out_v1, float *out_v2);
float BMS_ComputeOCV(float soc);
float BMS_ComputedOCV(float soc);

#ifdef __cplusplus
}
#endif

#endif /* BMS_EKF_H */
