/**
 * @file bms_ekf.h
 * @brief ANSI C 3-state 2-RC thermal EKF with diagnostics for an automotive BMS.
 *
 * This is the C twin of simulink_matlab/lib/bms_est_step.m (estimator mode 0: fixed, ACO-tuned Q and R).
 * Same equations, same order of operations, same limits; only the arithmetic is 32-bit float.
 * simulink_matlab/lib/bms_sil_run.m proves the equivalence by feeding both the same measurements.
 *
 * Target: any 32-bit MCU with a float library (ARM Cortex-M, STM32, TI C2000).
 * Memory: no dynamic allocation; the whole estimator is one BMS_EKF_t object (see sizeof).
 * Units : SI. Current positive when discharging. Temperature in kelvin. SOC as a fraction 0..1
 *         inside the filter; the output struct reports it in percent.
 */

#ifndef BMS_EKF_H
#define BMS_EKF_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <math.h>

#define BMS_STATE_DIM    3       /* x = [SOC, V1, V2]^T */
#define BMS_MEAS_DIM     1       /* y = [Vt]            */
#define BMS_OCV_COEFFS   9       /* OCV polynomial of order 8, highest power first */

/* Diagnostic code: bit mask, several bits can be set at once (same values as the MATLAB library). */
#define BMS_FAULT_V_RANGE      1u    /* terminal voltage outside the plausible window          */
#define BMS_FAULT_T_RANGE      2u    /* temperature outside the plausible window (last good T used) */
#define BMS_FAULT_I_RANGE      4u    /* current magnitude above the limit                      */
#define BMS_FAULT_REJECTED     8u    /* innovation rejected by the NIS gate                    */
#define BMS_FAULT_INCONSISTENT 16u   /* running mean NIS too high: model/noise mismatch        */

typedef struct {
    float R0_ref;   /* Ohmic resistance at the reference temperature [Ohm]  */
    float R1_ref;   /* Fast polarisation resistance [Ohm]                   */
    float C1;       /* Fast polarisation capacitance [Farad]                */
    float R2_ref;   /* Slow polarisation resistance [Ohm]                   */
    float C2;       /* Slow polarisation capacitance [Farad]                */
    float Qn;       /* Nominal capacity [Ampere-seconds]                    */
    float eta;      /* Coulombic efficiency                                 */
    float dt;       /* Sampling period [s]                                  */
    float Ea;       /* Arrhenius activation energy [J/mol]                  */
    float T_ref;    /* Reference temperature [K]                            */
} BMS_BatteryParams_t;

typedef struct {
    float q_SOC;                 /* process-noise variance, SOC state     */
    float q_V1;                  /* process-noise variance, V1 state      */
    float q_V2;                  /* process-noise variance, V2 state      */
    float r_V;                   /* measurement-noise variance, voltage   */
    float p0_SOC;                /* initial covariance, SOC               */
    float p0_V1;                 /* initial covariance, V1                */
    float p0_V2;                 /* initial covariance, V2                */
    float ocv[BMS_OCV_COEFFS];   /* OCV(SOC) polynomial, highest power first */
} BMS_EKF_Config_t;

typedef struct {
    float    v_min, v_max;       /* plausible terminal voltage [V]        */
    float    t_min, t_max;       /* plausible temperature [K]             */
    float    i_max;              /* plausible current magnitude [A]       */
    float    gate_nis;           /* reject a sample if NIS exceeds this   */
    uint16_t gate_k0;            /* gate arms after this many samples     */
    uint16_t max_rej;            /* force-accept after this many rejects in a row */
    float    nis_alpha;          /* consistency monitor smoothing factor  */
    float    nis_limit;          /* flag if smoothed NIS exceeds this     */
} BMS_Diag_Config_t;

typedef struct {
    float   soc_pct;             /* state of charge, percent              */
    float   v_est;               /* predicted terminal voltage [V]        */
    float   innovation;          /* measured minus predicted voltage [V]  */
    float   nis;                 /* normalised innovation squared         */
    float   p_trace;             /* trace of the covariance after update  */
    uint8_t fault;               /* BMS_FAULT_* bit mask                  */
} BMS_EKF_Out_t;

typedef struct {
    BMS_BatteryParams_t params;
    BMS_EKF_Config_t    config;
    BMS_Diag_Config_t   diag;

    float    x_hat[BMS_STATE_DIM];                 /* state [SOC, V1, V2]            */
    float    P[BMS_STATE_DIM][BMS_STATE_DIM];      /* covariance                     */
    float    docv[BMS_OCV_COEFFS - 1];             /* dOCV/dSOC polynomial (derived) */

    float    T_last;                               /* last plausible temperature     */
    float    nis_avg;                              /* smoothed NIS                   */
    uint32_t k;                                    /* samples processed              */
    uint16_t rej;                                  /* consecutive rejected samples   */
    uint8_t  is_initialized;
} BMS_EKF_t;

/** Initialise with the default cell, ACO-tuned noise settings and diagnostics. */
void BMS_EKF_Init(BMS_EKF_t *ekf, float init_soc, float temp_k);

/** Initialise with caller-supplied cell parameters, noise settings and diagnostic limits. */
void BMS_EKF_Configure(BMS_EKF_t *ekf, const BMS_BatteryParams_t *params, const BMS_EKF_Config_t *config,
                       const BMS_Diag_Config_t *diag, float init_soc);

/** One filter step with full output (call once per sampling period). */
void BMS_EKF_StepEx(BMS_EKF_t *ekf, float current_A, float voltage_meas_V, float temp_k, BMS_EKF_Out_t *out);

/** Original interface: SOC, V1 and V2 as fractions/volts. */
void BMS_EKF_Step(BMS_EKF_t *ekf, float current_A, float voltage_meas_V, float temp_k,
                  float *out_soc, float *out_v1, float *out_v2);

/** OCV(SOC) and its slope for the default curve (slope floored at 0.02 V per unit SOC). */
float BMS_ComputeOCV(float soc);
float BMS_ComputedOCV(float soc);

#ifdef __cplusplus
}
#endif

#endif /* BMS_EKF_H */
