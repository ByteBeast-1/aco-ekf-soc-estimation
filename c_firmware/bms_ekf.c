/**
 * @file bms_ekf.c
 * @brief Production ANSI C Implementation of 3-State 2-RC Thermal EKF.
 */

#include "bms_ekf.h"

static const float ocv_poly[9] = {
    -21.7497f,  82.2625f, -125.3296f,  99.6313f, -44.5010f,  11.3416f,  -1.5825f,   1.5032f,   3.0130f
};

static const float docv_poly[8] = {
    -173.9976f, 575.8375f, -751.9776f, 498.1565f, -178.0040f,  34.0248f,  -3.1650f,   1.5032f
};

float BMS_ComputeOCV(float soc) {
    if (soc < 0.0f) soc = 0.0f;
    if (soc > 1.0f) soc = 1.0f;
    float val = ocv_poly[0];
    for (int i = 1; i < 9; i++) {
        val = val * soc + ocv_poly[i];
    }
    return val;
}

float BMS_ComputedOCV(float soc) {
    if (soc < 0.0f) soc = 0.0f;
    if (soc > 1.0f) soc = 1.0f;
    float val = docv_poly[0];
    for (int i = 1; i < 8; i++) {
        val = val * soc + docv_poly[i];
    }
    return val;
}

void BMS_EKF_Init(BMS_EKF_t *ekf, float init_soc, float temp_k) {
    (void)temp_k;
    ekf->params.R0_ref = 0.012f;
    ekf->params.R1_ref = 0.008f;
    ekf->params.C1     = 1500.0f;
    ekf->params.R2_ref = 0.015f;
    ekf->params.C2     = 12000.0f;
    ekf->params.Qn     = 7200.0f;
    ekf->params.eta    = 1.0f;
    ekf->params.dt     = 1.0f;
    ekf->params.Ea     = 18000.0f;
    ekf->params.T_ref  = 298.15f;

    ekf->config.q_SOC = 1.00e-7f;
    ekf->config.q_V1  = 1.00e-7f;
    ekf->config.q_V2  = 1.00e-7f;
    ekf->config.r_V   = 2.455e-3f;

    ekf->x_hat[0] = init_soc;
    ekf->x_hat[1] = 0.0f;
    ekf->x_hat[2] = 0.0f;

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            ekf->P[i][j] = (i == j) ? ((i == 0) ? 0.01f : 0.0025f) : 0.0f;
        }
    }
    ekf->is_initialized = 1;
}

void BMS_EKF_Step(BMS_EKF_t *ekf, float current_A, float voltage_meas_V, float temp_k, float *out_soc, float *out_v1, float *out_v2) {
    if (!ekf->is_initialized) return;

    /* Thermal adaptation */
    float tfactor = expf((ekf->params.Ea / 8.314f) * (1.0f / temp_k - 1.0f / ekf->params.T_ref));
    float R0 = ekf->params.R0_ref * tfactor;
    float R1 = ekf->params.R1_ref * tfactor;
    float R2 = ekf->params.R2_ref * tfactor;

    float a1 = expf(-ekf->params.dt / (R1 * ekf->params.C1));
    float b1 = R1 * (1.0f - a1);
    float a2 = expf(-ekf->params.dt / (R2 * ekf->params.C2));
    float b2 = R2 * (1.0f - a2);

    /* 1. Predict state */
    float soc_p = ekf->x_hat[0] - (ekf->params.eta * current_A * ekf->params.dt) / ekf->params.Qn;
    if (soc_p < 0.0f) soc_p = 0.0f;
    if (soc_p > 1.0f) soc_p = 1.0f;
    float v1_p = a1 * ekf->x_hat[1] + b1 * current_A;
    float v2_p = a2 * ekf->x_hat[2] + b2 * current_A;

    /* 2. Predict covariance P_pred = F * P * F^T + Q */
    float F[3][3] = { {1.0f, 0.0f, 0.0f}, {0.0f, a1, 0.0f}, {0.0f, 0.0f, a2} };
    float P_pred[3][3];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            float sum = 0.0f;
            for (int k = 0; k < 3; k++) {
                for (int l = 0; l < 3; l++) {
                    sum += F[i][k] * ekf->P[k][l] * F[j][l];
                }
            }
            if (i == j) {
                float q_val = (i == 0) ? ekf->config.q_SOC : ((i == 1) ? ekf->config.q_V1 : ekf->config.q_V2);
                sum += q_val;
            }
            P_pred[i][j] = sum;
        }
    }

    /* 3. Predict measurement */
    float ocv = BMS_ComputeOCV(soc_p);
    float v_pred = ocv - v1_p - v2_p - (R0 * current_A);
    float residual = voltage_meas_V - v_pred;

    /* 4. Measurement Jacobian H = [docv, -1, -1] */
    float dOCV = BMS_ComputedOCV(soc_p);
    float H[3] = { dOCV, -1.0f, -1.0f };

    /* 5. Innovation covariance S = H * P_pred * H^T + R */
    float S = ekf->config.r_V;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            S += H[i] * P_pred[i][j] * H[j];
        }
    }

    /* 6. Kalman gain K = (P_pred * H^T) / S */
    float K[3];
    for (int i = 0; i < 3; i++) {
        float sum = 0.0f;
        for (int j = 0; j < 3; j++) {
            sum += P_pred[i][j] * H[j];
        }
        K[i] = sum / S;
    }

    /* 7. Correct State */
    ekf->x_hat[0] = soc_p + K[0] * residual;
    if (ekf->x_hat[0] < 0.0f) ekf->x_hat[0] = 0.0f;
    if (ekf->x_hat[0] > 1.0f) ekf->x_hat[0] = 1.0f;
    ekf->x_hat[1] = v1_p + K[1] * residual;
    ekf->x_hat[2] = v2_p + K[2] * residual;

    /* 8. Joseph-Form Covariance Update P_upd = (I - K*H)*P_pred*(I - K*H)^T + K*R*K^T */
    float IKH[3][3];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            float eye_val = (i == j) ? 1.0f : 0.0f;
            IKH[i][j] = eye_val - K[i] * H[j];
        }
    }

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            float sum = 0.0f;
            for (int k = 0; k < 3; k++) {
                for (int l = 0; l < 3; l++) {
                    sum += IKH[i][k] * P_pred[k][l] * IKH[j][l];
                }
            }
            sum += K[i] * ekf->config.r_V * K[j];
            ekf->P[i][j] = sum;
        }
    }

    if (out_soc) *out_soc = ekf->x_hat[0];
    if (out_v1)  *out_v1  = ekf->x_hat[1];
    if (out_v2)  *out_v2  = ekf->x_hat[2];
}
