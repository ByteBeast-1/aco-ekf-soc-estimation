/**
 * @file bms_ekf.h
 * @brief Production ANSI C Extended Kalman Filter (EKF) for Automotive BMS SOC Estimation.
 * @author BMS Firmware Engineering Team
 *
 * Target: ARM Cortex-M0+/M3/M4/M7, STM32, Texas Instruments C2000 / TMS570
 * RAM Usage: < 1.5 KB (zero dynamic memory allocation - no malloc)
 * Execution Time: < 0.5 ms per sample at 1 Hz update rate
 */

#ifndef BMS_EKF_H
#define BMS_EKF_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <math.h>

/* ========================================================================= */
/* Configuration & Constants                                                 */
/* ========================================================================= */
#define BMS_STATE_DIM       2       /* x = [SOC, Vp]^T */
#define BMS_MEAS_DIM        1       /* y = [Vt]        */

/* Battery Parameters (Default 2 Ah LFP/NMC Cell) */
typedef struct {
    float R0;       /* Ohmic resistance [Ohm]                     */
    float R1;       /* Polarization resistance [Ohm]              */
    float C1;       /* Polarization capacitance [Farad]           */
    float Qn;       /* Nominal Capacity [Ampere-seconds] (As)     */
    float eta;      /* Coulombic efficiency                       */
    float dt;       /* Sampling period [seconds]                  */
} BMS_BatteryParams_t;

/* EKF Covariance Settings (Tuned by ACO) */
typedef struct {
    float q_SOC;    /* Process noise variance for SOC state       */
    float q_Vp;     /* Process noise variance for Vp state        */
    float r_V;      /* Measurement noise variance for Voltage     */
} BMS_EKF_Config_t;

/* EKF Controller Instance Handle */
typedef struct {
    BMS_BatteryParams_t params;
    BMS_EKF_Config_t    config;

    float x_hat[BMS_STATE_DIM];                 /* State vector [SOC, Vp] */
    float P[BMS_STATE_DIM][BMS_STATE_DIM];      /* Covariance matrix P    */
    
    float a;        /* Pre-computed discrete RC decay factor  */
    float b;        /* Pre-computed discrete RC gain factor   */

    uint8_t is_initialized;
} BMS_EKF_t;

/* ========================================================================= */
/* Function Declarations                                                     */
/* ========================================================================= */

/**
 * @brief Initialize the EKF instance with battery model and ACO-tuned covariances.
 * @param handle Pointer to BMS_EKF_t handle.
 * @param params Pointer to battery physical parameter structure.
 * @param config Pointer to ACO-tuned covariance configuration.
 * @param initial_soc Initial State of Charge estimate (0.0 to 1.0).
 */
void BMS_EKF_Init(BMS_EKF_t *handle, const BMS_BatteryParams_t *params, const BMS_EKF_Config_t *config, float initial_soc);

/**
 * @brief Execute one EKF update cycle (Predict -> Innovate -> Correct).
 * @param handle Pointer to BMS_EKF_t handle.
 * @param current_A Measured current in Amperes (Positive = Discharge, Negative = Charge).
 * @param voltage_V Measured terminal voltage in Volts.
 * @return float Corrected State of Charge (SOC) estimate [0.0 to 1.0].
 */
float BMS_EKF_Update(BMS_EKF_t *handle, float current_A, float voltage_V);

/**
 * @brief Polynomial evaluation for Open Circuit Voltage OCV(SOC).
 * @param soc Current State of Charge in range [0.0, 1.0].
 * @return float Open Circuit Voltage in Volts.
 */
float BMS_Get_OCV(float soc);

/**
 * @brief Derivative of OCV polynomial dOCV/dSOC for Measurement Jacobian H.
 * @param soc Current State of Charge in range [0.0, 1.0].
 * @return float Derivative dOCV/dSOC.
 */
float BMS_Get_dOCV_dSOC(float soc);

#ifdef __cplusplus
}
#endif

#endif /* BMS_EKF_H */
