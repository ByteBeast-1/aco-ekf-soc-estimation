/**
 * @file bms_ekf.c
 * @brief ANSI C implementation of the 3-state 2-RC thermal EKF with diagnostics.
 *
 * Mirrors simulink_matlab/lib/bms_est_step.m (mode 0). Equation numbers refer to the project guide.
 */

#include "bms_ekf.h"

#define BMS_R_GAS 8.314f

/* Default cell, identical to simulink_matlab/lib/bms_params.m('synthetic') */
static const float k_default_ocv[BMS_OCV_COEFFS] = {
    -21.7497f, 82.2625f, -125.3296f, 99.6313f, -44.5010f, 11.3416f, -1.5825f, 1.5032f, 3.0130f
};

/* ------------------------------------------------------------------------- helpers */

static float clampf(float x, float lo, float hi)
{
    if (x < lo) { return lo; }
    if (x > hi) { return hi; }
    return x;
}

/* Horner evaluation, highest power first: one multiply and one add per coefficient. */
static float poly_eval(const float *c, int n, float x)
{
    float v = c[0];
    int i;
    for (i = 1; i < n; i++) {
        v = v * x + c[i];
    }
    return v;
}

static void derive_docv(BMS_EKF_t *ekf)
{
    int i;
    for (i = 0; i < BMS_OCV_COEFFS - 1; i++) {
        ekf->docv[i] = ekf->config.ocv[i] * (float)(BMS_OCV_COEFFS - 1 - i);
    }
}

/* OCV and slope for a SOC, clamped to 0..1; slope floored so the Kalman gain never collapses. */
static void ocv_and_slope(const BMS_EKF_t *ekf, float soc, float *ocv, float *dv)
{
    float s = clampf(soc, 0.0f, 1.0f);
    float d;
    *ocv = poly_eval(ekf->config.ocv, BMS_OCV_COEFFS, s);
    d    = poly_eval(ekf->docv, BMS_OCV_COEFFS - 1, s);
    *dv  = (d > 0.02f) ? d : 0.02f;
}

float BMS_ComputeOCV(float soc)
{
    return poly_eval(k_default_ocv, BMS_OCV_COEFFS, clampf(soc, 0.0f, 1.0f));
}

float BMS_ComputedOCV(float soc)
{
    float d[BMS_OCV_COEFFS - 1];
    float dv;
    int i;
    for (i = 0; i < BMS_OCV_COEFFS - 1; i++) {
        d[i] = k_default_ocv[i] * (float)(BMS_OCV_COEFFS - 1 - i);
    }
    dv = poly_eval(d, BMS_OCV_COEFFS - 1, clampf(soc, 0.0f, 1.0f));
    return (dv > 0.02f) ? dv : 0.02f;
}

/* ------------------------------------------------------------------------- init */

void BMS_EKF_Configure(BMS_EKF_t *ekf, const BMS_BatteryParams_t *params, const BMS_EKF_Config_t *config,
                       const BMS_Diag_Config_t *diag, float init_soc)
{
    int i, j;
    ekf->params = *params;
    ekf->config = *config;
    ekf->diag   = *diag;
    derive_docv(ekf);

    ekf->x_hat[0] = init_soc;
    ekf->x_hat[1] = 0.0f;
    ekf->x_hat[2] = 0.0f;
    for (i = 0; i < BMS_STATE_DIM; i++) {
        for (j = 0; j < BMS_STATE_DIM; j++) {
            ekf->P[i][j] = 0.0f;
        }
    }
    ekf->P[0][0] = config->p0_SOC;
    ekf->P[1][1] = config->p0_V1;
    ekf->P[2][2] = config->p0_V2;

    ekf->T_last    = 298.15f;
    ekf->nis_avg   = 1.0f;
    ekf->innov_avg = 0.0f;
    ekf->k         = 0u;
    ekf->rej       = 0u;
    ekf->ibias_cnt = 0u;
    ekf->is_initialized = 1u;
}

void BMS_EKF_Init(BMS_EKF_t *ekf, float init_soc, float temp_k)
{
    BMS_BatteryParams_t p;
    BMS_EKF_Config_t    c;
    BMS_Diag_Config_t   d;
    int i;
    (void)temp_k;

    p.R0_ref = 0.012f;  p.R1_ref = 0.008f;  p.R2_ref = 0.015f;
    p.C1 = 1500.0f;     p.C2 = 12000.0f;    p.Qn = 7200.0f;
    p.eta = 1.0f;       p.dt = 1.0f;        p.Ea = 18000.0f;   p.T_ref = 298.15f;

    /* ACO-tuned noise settings (simulink_matlab/lib/bms_tuned_data.m, synthetic cell) */
    c.q_SOC = 1.000000e-09f;  c.q_V1 = 1.000000e-09f;  c.q_V2 = 1.533887e-07f;
    c.r_V   = 1.474701e-03f;
    c.p0_SOC = 0.01f;  c.p0_V1 = 0.0025f;  c.p0_V2 = 0.0025f;
    for (i = 0; i < BMS_OCV_COEFFS; i++) { c.ocv[i] = k_default_ocv[i]; }

    d.v_min = 2.0f;  d.v_max = 4.8f;  d.t_min = 250.0f;  d.t_max = 345.0f;  d.i_max = 10.0f;
    d.gate_nis = 20.0f;  d.gate_k0 = 30u;  d.max_rej = 10u;
    d.nis_alpha = 0.02f;  d.nis_limit = 5.0f;
    d.ibias_alpha = 0.02f;  d.ibias_thresh = 0.0055f;
    d.ibias_k0 = 150u;  d.ibias_count_limit = 20u;

    BMS_EKF_Configure(ekf, &p, &c, &d, init_soc);
}

/* ------------------------------------------------------------------------- step */

void BMS_EKF_StepEx(BMS_EKF_t *ekf, float current_A, float voltage_meas_V, float temp_k, BMS_EKF_Out_t *out)
{
    float F[BMS_STATE_DIM];                      /* diagonal of the state-transition matrix */
    float Pp[BMS_STATE_DIM][BMS_STATE_DIM];
    float Pu[BMS_STATE_DIM][BMS_STATE_DIM];
    float xp[BMS_STATE_DIM], xu[BMS_STATE_DIM];
    float Hm[BMS_STATE_DIM], hp[BMS_STATE_DIM], K[BMS_STATE_DIM];
    float IKH[BMS_STATE_DIM][BMS_STATE_DIM], tmp[BMS_STATE_DIM][BMS_STATE_DIM];
    float qd[BMS_STATE_DIM];
    float T, Tc, tf, R0, R1, R2, a1, b1, a2, b2;
    float ocv, dv, vest, r, S, nis, sq;
    unsigned int flags = 0u;
    int vbad, accept;
    int i, j, l;

    if (!ekf->is_initialized) { return; }
    ekf->k += 1u;

    /* 1. sensor plausibility */
    T = temp_k;
    vbad = !((voltage_meas_V > ekf->diag.v_min) && (voltage_meas_V < ekf->diag.v_max));
    if (vbad) { flags |= BMS_FAULT_V_RANGE; }
    if (!((T > ekf->diag.t_min) && (T < ekf->diag.t_max))) {
        flags |= BMS_FAULT_T_RANGE;
        T = ekf->T_last;                         /* hold last good temperature */
    } else {
        ekf->T_last = T;
    }
    if (fabsf(current_A) > ekf->diag.i_max) { flags |= BMS_FAULT_I_RANGE; }

    /* 2. thermal adaptation and RC coefficients (Arrhenius law) */
    Tc = clampf(T, 250.0f, 350.0f);
    tf = expf((ekf->params.Ea / BMS_R_GAS) * (1.0f / Tc - 1.0f / ekf->params.T_ref));
    R0 = ekf->params.R0_ref * tf;
    R1 = ekf->params.R1_ref * tf;
    R2 = ekf->params.R2_ref * tf;
    a1 = expf(-ekf->params.dt / (R1 * ekf->params.C1));
    b1 = R1 * (1.0f - a1);
    a2 = expf(-ekf->params.dt / (R2 * ekf->params.C2));
    b2 = R2 * (1.0f - a2);

    /* 3. time update: state and covariance */
    xp[0] = clampf(ekf->x_hat[0] - ekf->params.eta * current_A * ekf->params.dt / ekf->params.Qn, 0.0f, 1.0f);
    xp[1] = a1 * ekf->x_hat[1] + b1 * current_A;
    xp[2] = a2 * ekf->x_hat[2] + b2 * current_A;
    F[0] = 1.0f;  F[1] = a1;  F[2] = a2;
    qd[0] = ekf->config.q_SOC;  qd[1] = ekf->config.q_V1;  qd[2] = ekf->config.q_V2;
    for (i = 0; i < BMS_STATE_DIM; i++) {
        for (j = 0; j < BMS_STATE_DIM; j++) {
            Pp[i][j] = (F[i] * ekf->P[i][j]) * F[j];
        }
        Pp[i][i] += qd[i];
    }

    /* 4. innovation: predicted voltage, residual, innovation covariance, NIS */
    ocv_and_slope(ekf, xp[0], &ocv, &dv);
    vest = ocv - xp[1] - xp[2] - R0 * current_A;
    r    = voltage_meas_V - vest;
    Hm[0] = dv;  Hm[1] = -1.0f;  Hm[2] = -1.0f;
    S = 0.0f;
    for (j = 0; j < BMS_STATE_DIM; j++) {
        hp[j] = 0.0f;
        for (i = 0; i < BMS_STATE_DIM; i++) { hp[j] += Hm[i] * Pp[i][j]; }
        S += hp[j] * Hm[j];
    }
    S += ekf->config.r_V;
    nis = (r * r) / S;

    /* 5. outlier gate: skip the correction for implausible samples */
    accept = !vbad;
    if (accept && ekf->k > (uint32_t)ekf->diag.gate_k0 && nis > ekf->diag.gate_nis && ekf->rej < ekf->diag.max_rej) {
        accept = 0;
        flags |= BMS_FAULT_REJECTED;
        ekf->rej += 1u;
    } else if (accept) {
        ekf->rej = 0u;
    }

    /* 6. measurement update, Joseph form */
    if (accept) {
        for (i = 0; i < BMS_STATE_DIM; i++) {
            float sum = 0.0f;
            for (j = 0; j < BMS_STATE_DIM; j++) { sum += Pp[i][j] * Hm[j]; }
            K[i] = sum / S;
        }
        for (i = 0; i < BMS_STATE_DIM; i++) { xu[i] = xp[i] + K[i] * r; }
        xu[0] = clampf(xu[0], 0.0f, 1.0f);

        for (i = 0; i < BMS_STATE_DIM; i++) {
            for (j = 0; j < BMS_STATE_DIM; j++) {
                IKH[i][j] = ((i == j) ? 1.0f : 0.0f) - K[i] * Hm[j];
            }
        }
        for (i = 0; i < BMS_STATE_DIM; i++) {
            for (j = 0; j < BMS_STATE_DIM; j++) {
                float sum = 0.0f;
                for (l = 0; l < BMS_STATE_DIM; l++) { sum += IKH[i][l] * Pp[l][j]; }
                tmp[i][j] = sum;
            }
        }
        for (i = 0; i < BMS_STATE_DIM; i++) {
            for (j = 0; j < BMS_STATE_DIM; j++) {
                float sum = 0.0f;
                for (l = 0; l < BMS_STATE_DIM; l++) { sum += tmp[i][l] * IKH[j][l]; }
                Pu[i][j] = sum + (K[i] * ekf->config.r_V) * K[j];
            }
        }
    } else {
        for (i = 0; i < BMS_STATE_DIM; i++) {
            xu[i] = xp[i];
            for (j = 0; j < BMS_STATE_DIM; j++) { Pu[i][j] = Pp[i][j]; }
        }
    }

    /* 7. consistency monitor */
    sq = (nis < 100.0f) ? nis : 100.0f;
    ekf->nis_avg = (1.0f - ekf->diag.nis_alpha) * ekf->nis_avg + ekf->diag.nis_alpha * sq;
    if (ekf->k > 100u && ekf->nis_avg > ekf->diag.nis_limit) { flags |= BMS_FAULT_INCONSISTENT; }

    /* 7b. current-bias monitor (bit 32) */
    if (ekf->k > (uint32_t)ekf->diag.ibias_k0 && xu[0] >= 0.08f && xu[0] <= 0.95f) {
        if (current_A >= 0.0f) {
            ekf->innov_avg = (1.0f - ekf->diag.ibias_alpha) * ekf->innov_avg + ekf->diag.ibias_alpha * r;
            if (fabsf(ekf->innov_avg) > ekf->diag.ibias_thresh) {
                ekf->ibias_cnt += 1u;
            } else if (ekf->ibias_cnt > 0u) {
                ekf->ibias_cnt -= 1u;
            }
        } else if (ekf->ibias_cnt > 0u) {
            ekf->ibias_cnt -= 1u;
        }
        if (ekf->ibias_cnt >= ekf->diag.ibias_count_limit) {
            flags |= BMS_FAULT_I_OFFSET;
        }
    }

    /* 8. commit state */
    for (i = 0; i < BMS_STATE_DIM; i++) {
        ekf->x_hat[i] = xu[i];
        for (j = 0; j < BMS_STATE_DIM; j++) { ekf->P[i][j] = Pu[i][j]; }
    }

    if (out) {
        out->soc_pct    = 100.0f * xu[0];
        out->v_est      = vest;
        out->innovation = r;
        out->nis        = nis;
        out->p_trace    = Pu[0][0] + Pu[1][1] + Pu[2][2];
        out->fault      = (uint8_t)flags;
    }
}

void BMS_EKF_Step(BMS_EKF_t *ekf, float current_A, float voltage_meas_V, float temp_k,
                  float *out_soc, float *out_v1, float *out_v2)
{
    BMS_EKF_Out_t o;
    BMS_EKF_StepEx(ekf, current_A, voltage_meas_V, temp_k, &o);
    if (out_soc) { *out_soc = ekf->x_hat[0]; }
    if (out_v1)  { *out_v1  = ekf->x_hat[1]; }
    if (out_v2)  { *out_v2  = ekf->x_hat[2]; }
}
