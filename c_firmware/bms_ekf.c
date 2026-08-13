/**
 * @file bms_ekf.c
 * @brief Implementation of Automotive BMS Extended Kalman Filter (EKF) in ANSI C.
 */

#include "bms_ekf.h"

/* 8th-order OCV Polynomial Coefficients */
static const float OCV_COEFFS[9] = {
    -21.7497f,  82.2625f, -125.3296f,  99.6313f,
    -44.5010f,  11.3416f,   -1.5825f,   1.5032f,   3.0130f
};

/* Derivative Polynomial Coefficients for dOCV/dSOC */
static const float dOCV_COEFFS[8] = {
    -173.9976f,  575.8375f, -751.9776f, 498.1565f,
    -178.0040f,   34.0248f,   -3.1650f,   1.5032f
};

/* Helper: Clamp float to range */
static inline float clampf(float val, float min_val, float max_val) {
    if (val < min_val) return min_val;
    if (val > max_val) return max_val;
    return val;
}

float BMS_Get_OCV(float soc) {
    soc = clampf(soc, 0.0f, 1.0f);
    float ocv = OCV_COEFFS[0];
    for (int i = 1; i < 9; i++) {
        ocv = ocv * soc + OCV_COEFFS[i];
    }
    return ocv;
}

float BMS_Get_dOCV_dSOC(float soc) {
    soc = clampf(soc, 0.0f, 1.0f);
    float docv = dOCV_COEFFS[0];
    for (int i = 1; i < 8; i++) {
        docv = docv * soc + dOCV_COEFFS[i];
    }
    return docv;
}

void BMS_EKF_Init(BMS_EKF_t *handle, const BMS_BatteryParams_t *params, const BMS_EKF_Config_t *config, float initial_soc) {
    if (!handle || !params || !config) return;

    handle->params = *params;
    handle->config = *config;

    /* Pre-compute discrete RC decay & gain */
    handle->a = expf(-params->dt / (params->R1 * params->C1));
    handle->b = params->R1 * (1.0f - handle->a);

    /* Initial state estimate [SOC, Vp] */
    handle->x_hat[0] = clampf(initial_soc, 0.0f, 1.0f);
    handle->x_hat[1] = 0.0f;

    /* Initial Covariance Matrix P0 */
    handle->P[0][0] = 0.01f;   /* 0.1^2 */
    handle->P[0][1] = 0.0f;
    handle->P[1][0] = 0.0f;
    handle->P[1][1] = 0.0025f; /* 0.05^2 */

    handle->is_initialized = 1;
}

float BMS_EKF_Update(BMS_EKF_t *handle, float current_A, float voltage_V) {
    if (!handle || !handle->is_initialized) return 0.0f;

    float dt  = handle->params.dt;
    float Qn  = handle->params.Qn;
    float eta = handle->params.eta;

    /* ── Step 1: Predict State x_pred ───────────────────────────────── */
    float soc_pred = handle->x_hat[0] - (eta * current_A * dt) / Qn;
    float vp_pred  = handle->a * handle->x_hat[1] + handle->b * current_A;
    soc_pred = clampf(soc_pred, 0.0f, 1.0f);

    /* ── Step 2: Predict Covariance P_pred = F * P * F^T + Q ────────── */
    float a = handle->a;
    float P_pred[2][2];
    P_pred[0][0] = handle->P[0][0] + handle->config.q_SOC;
    P_pred[0][1] = handle->P[0][1] * a;
    P_pred[1][0] = handle->P[1][0] * a;
    P_pred[1][1] = a * a * handle->P[1][1] + handle->config.q_Vp;

    /* ── Step 3: Predict Measurement Vt_pred & Innovation r ─────────── */
    float ocv_pred = BMS_Get_OCV(soc_pred);
    float vt_pred  = ocv_pred - vp_pred - handle->params.R0 * current_A;
    float r = voltage_V - vt_pred;  /* Innovation residual */

    /* ── Step 4: Measurement Jacobian H & Innovation Covariance S ───── */
    float docv = BMS_Get_dOCV_dSOC(soc_pred);
    float H[2] = { docv, -1.0f };   /* 1x2 measurement Jacobian */

    /* S = H * P_pred * H^T + R */
    float S = H[0] * (P_pred[0][0] * H[0] + P_pred[1][0] * H[1]) +
              H[1] * (P_pred[0][1] * H[0] + P_pred[1][1] * H[1]) +
              handle->config.r_V;

    if (fabsf(S) < 1e-9f) S = 1e-9f;

    /* ── Step 5: Compute Kalman Gain K = (P_pred * H^T) / S ─────────── */
    float K[2];
    K[0] = (P_pred[0][0] * H[0] + P_pred[0][1] * H[1]) / S;
    K[1] = (P_pred[1][0] * H[0] + P_pred[1][1] * H[1]) / S;

    /* ── Step 6: Correct State x_upd = x_pred + K * r ───────────────── */
    handle->x_hat[0] = clampf(soc_pred + K[0] * r, 0.0f, 1.0f);
    handle->x_hat[1] = vp_pred + K[1] * r;

    /* ── Step 7: Joseph-Form Covariance Correction ──────────────────── */
    /* IKH = I - K*H */
    float IKH[2][2];
    IKH[0][0] = 1.0f - K[0] * H[0];
    IKH[0][1] = -K[0] * H[1];
    IKH[1][0] = -K[1] * H[0];
    IKH[1][1] = 1.0f - K[1] * H[1];

    /* P_upd = IKH * P_pred * IKH^T + K * R * K^T */
    float tmp[2][2];
    tmp[0][0] = IKH[0][0] * P_pred[0][0] + IKH[0][1] * P_pred[1][0];
    tmp[0][1] = IKH[0][0] * P_pred[0][1] + IKH[0][1] * P_pred[1][1];
    tmp[1][0] = IKH[1][0] * P_pred[0][0] + IKH[1][1] * P_pred[1][0];
    tmp[1][1] = IKH[1][0] * P_pred[0][1] + IKH[1][1] * P_pred[1][1];

    float R_val = handle->config.r_V;
    handle->P[0][0] = tmp[0][0] * IKH[0][0] + tmp[0][1] * IKH[0][1] + K[0] * K[0] * R_val;
    handle->P[0][1] = tmp[0][0] * IKH[1][0] + tmp[0][1] * IKH[1][1] + K[0] * K[1] * R_val;
    handle->P[1][0] = tmp[1][0] * IKH[0][0] + tmp[1][1] * IKH[0][1] + K[1] * K[0] * R_val;
    handle->P[1][1] = tmp[1][0] * IKH[1][0] + tmp[1][1] * IKH[1][1] + K[1] * K[1] * R_val;

    return handle->x_hat[0];
}
