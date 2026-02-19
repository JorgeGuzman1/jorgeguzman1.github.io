#include "EKF.hpp"
#include <cmath>
#include <cstring>

namespace aon {

EKF::EKF()
    : q_pos_(0.01), q_theta_(0.001), q_v_(0.1), q_omega_(0.1), distance_traveled_in_(0.0) {

    // Inicializar estado en origen
    state_ = {0.0, 0.0, 0.0, 0.0, 0.0};

    // Inicializar covarianza: diagonal con valores iniciales
    std::memset(P_, 0, sizeof(P_));
    P_[0][0] = 0.1;    // x
    P_[1][1] = 0.1;    // y
    P_[2][2] = 0.01;   // theta
    P_[3][3] = 0.5;    // v
    P_[4][4] = 0.5;    // omega
}

void EKF::predict_from_odometry(double v_inps, double omega_rad_s, double dt_s) {
    // Guardar estado anterior
    double x_prev = state_.x;
    double y_prev = state_.y;
    double theta_prev = state_.theta;

    // Predicción del estado (modelo cinemático unicycle)
    if (std::abs(omega_rad_s) < 1e-6) {
        // Movimiento aproximadamente rectilíneo
        state_.x += v_inps * std::cos(theta_prev) * dt_s;
        state_.y += v_inps * std::sin(theta_prev) * dt_s;
        state_.theta += omega_rad_s * dt_s;
    } else {
        // Movimiento con rotación
        double ratio = v_inps / omega_rad_s;
        state_.x += ratio * (std::sin(theta_prev + omega_rad_s * dt_s) - std::sin(theta_prev));
        state_.y += ratio * (-std::cos(theta_prev + omega_rad_s * dt_s) + std::cos(theta_prev));
        state_.theta += omega_rad_s * dt_s;
    }

    state_.v = v_inps;
    state_.omega = omega_rad_s;

    // Normalizar ángulo
    normalize_angle(state_.theta);

    // Acumular distancia
    distance_traveled_in_ += std::abs(v_inps * dt_s);

    // Predecir covarianza
    predict_covariance(v_inps, omega_rad_s, dt_s);
}

void EKF::correct_with_gyro(double yaw_rate_rad_s, double var_yaw_rate) {
    // Medición: [omega]
    double z[1] = {yaw_rate_rad_s};
    double R[1] = {var_yaw_rate};

    // Actualizar estado omega directamente (medición directa)
    state_.omega = yaw_rate_rad_s;

    // Actualizar covarianza
    update_covariance_sensor(z, R, 4); // sensor 4 = omega
}

void EKF::correct_with_encoders(double ds_in, double dtheta_rad, double var_ds, double var_dtheta) {
    // Medición: [ds, dtheta] - integra al estado actual
    state_.x += ds_in * std::cos(state_.theta + dtheta_rad/2);
    state_.y += ds_in * std::sin(state_.theta + dtheta_rad/2);
    state_.theta += dtheta_rad;
    state_.v = ds_in / 0.02; // asume dt=20ms, aproximar
    state_.omega = dtheta_rad / 0.02;

    normalize_angle(state_.theta);

    // Actualizar covarianza
    double z[2] = {ds_in, dtheta_rad};
    double R[2] = {var_ds, var_dtheta};
    update_covariance_sensor(z, R, 0); // sensor 0 = position correction
}

void EKF::correct_with_gps(double x_in, double y_in, double var_x, double var_y,
                          bool use_heading, double heading_deg, double var_heading) {

    double z[3] = {x_in, y_in, 0.0};
    double R[9] = {var_x, 0.0, 0.0,
                   0.0, var_y, 0.0,
                   0.0, 0.0, 1e6}; // muy grande para heading si no se usa

    if (use_heading) {
        z[2] = heading_deg * M_PI / 180.0;
        R[8] = var_heading;
    }

    // Actualizar posición (y opcionalmente heading)
    state_.x = x_in;
    state_.y = y_in;
    if (use_heading) {
        normalize_angle(z[2]);
        state_.theta = z[2];
    }

    // Actualizar covarianza
    update_covariance_gps(z, R, use_heading);
}

EKFState EKF::get_state() const {
    return state_;
}

double EKF::get_distance_traveled_in() const {
    return distance_traveled_in_;
}

void EKF::predict_covariance(double v, double omega, double dt) {
    // Jacobiano del proceso F = ∂f/∂x
    double F[5][5] = {{1.0, 0.0, 0.0, 0.0, 0.0},
                      {0.0, 1.0, 0.0, 0.0, 0.0},
                      {0.0, 0.0, 1.0, 0.0, dt},
                      {0.0, 0.0, 0.0, 1.0, 0.0},
                      {0.0, 0.0, 0.0, 0.0, 1.0}};

    // Términos dependientes del estado (derivadas de la cinemática)
    if (std::abs(omega) < 1e-6) {
        // Caso rectilíneo
        F[0][2] = -v * std::sin(state_.theta) * dt;
        F[0][3] = std::cos(state_.theta) * dt;
        F[1][2] = v * std::cos(state_.theta) * dt;
        F[1][3] = std::sin(state_.theta) * dt;
    } else {
        // Caso general con rotación
        double theta_new = state_.theta + omega * dt;
        double ratio = v / omega;

        F[0][2] = ratio * (std::cos(theta_new) - std::cos(state_.theta));
        F[0][3] = (std::sin(theta_new) - std::sin(state_.theta)) / omega;
        F[0][4] = v * (std::cos(theta_new) * dt - (std::sin(theta_new) - std::sin(state_.theta)) / (omega * omega));

        F[1][2] = ratio * (std::sin(theta_new) - std::sin(state_.theta));
        F[1][3] = (-std::cos(theta_new) + std::cos(state_.theta)) / omega;
        F[1][4] = v * (std::sin(theta_new) * dt - (-std::cos(theta_new) + std::cos(state_.theta)) / (omega * omega));

        F[2][4] = dt;
    }

    // Matriz de ruido del proceso Q
    double Q[5][5] = {{q_pos_, 0.0, 0.0, 0.0, 0.0},
                      {0.0, q_pos_, 0.0, 0.0, 0.0},
                      {0.0, 0.0, q_theta_, 0.0, 0.0},
                      {0.0, 0.0, 0.0, q_v_, 0.0},
                      {0.0, 0.0, 0.0, 0.0, q_omega_}};

    // Escalar ruido por dt
    for (int i = 0; i < 5; ++i) {
        Q[i][i] *= dt;
    }

    // P = F * P * F^T + Q (implementación simplificada)
    double P_new[5][5] = {{0}};

    // Calcular F * P * F^T
    for (int i = 0; i < 5; ++i) {
        for (int j = 0; j < 5; ++j) {
            for (int k = 0; k < 5; ++k) {
                P_new[i][j] += F[i][k] * P_[k][j];
            }
        }
    }

    double P_temp[5][5] = {{0}};
    for (int i = 0; i < 5; ++i) {
        for (int j = 0; j < 5; ++j) {
            for (int k = 0; k < 5; ++k) {
                P_temp[i][j] += P_new[i][k] * F[j][k];
            }
        }
    }

    // Sumar ruido Q
    for (int i = 0; i < 5; ++i) {
        for (int j = 0; j < 5; ++j) {
            P_[i][j] = P_temp[i][j] + Q[i][j];
        }
    }
}

void EKF::update_covariance_gps(const double* z, const double* R, bool use_heading) {
    // Matriz de medición H para GPS: mide [x, y, theta]
    int meas_dim = use_heading ? 3 : 2;
    double H[3][5] = {{1.0, 0.0, 0.0, 0.0, 0.0},
                      {0.0, 1.0, 0.0, 0.0, 0.0},
                      {0.0, 0.0, 1.0, 0.0, 0.0}};

    // S = H * P * H^T + R
    double S[3][3] = {{0}};
    for (int i = 0; i < meas_dim; ++i) {
        for (int j = 0; j < meas_dim; ++j) {
            for (int k = 0; k < 5; ++k) {
                S[i][j] += H[i][k] * P_[k][j];
            }
        }
    }

    double S_temp[3][3] = {{0}};
    for (int i = 0; i < meas_dim; ++i) {
        for (int j = 0; j < meas_dim; ++j) {
            for (int k = 0; k < 5; ++k) {
                S_temp[i][j] += S[i][k] * H[j][k];
            }
            S_temp[i][j] += R[i * 3 + j];
        }
    }

    // Calcular ganancia de Kalman K = P * H^T * S^(-1)
    // Implementación simplificada (asume S diagonal para inversión)
    double K[5][3] = {{0}};
    for (int i = 0; i < 5; ++i) {
        for (int j = 0; j < meas_dim; ++j) {
            for (int k = 0; k < 5; ++k) {
                K[i][j] += P_[i][k] * H[j][k];
            }
            if (S_temp[j][j] > 1e-6) {
                K[i][j] /= S_temp[j][j];
            }
        }
    }

    // Actualizar covarianza: P = (I - K * H) * P
    for (int i = 0; i < 5; ++i) {
        for (int j = 0; j < 5; ++j) {
            double kh_term = 0.0;
            for (int k = 0; k < meas_dim; ++k) {
                kh_term += K[i][k] * H[k][j];
            }
            P_[i][j] *= (1.0 - kh_term);
        }
    }
}

void EKF::update_covariance_sensor(const double* z, const double* R, int sensor_type) {
    // Simplificación: reducir varianza directamente para mediciones directas
    switch (sensor_type) {
        case 4: // omega
            if (R[0] > 1e-6) {
                double gain = P_[4][4] / (P_[4][4] + R[0]);
                P_[4][4] *= (1.0 - gain);
            }
            break;
        case 0: // position correction
            if (R[0] > 1e-6 && R[1] > 1e-6) {
                double gain_x = P_[0][0] / (P_[0][0] + R[0]);
                double gain_theta = P_[2][2] / (P_[2][2] + R[1]);
                P_[0][0] *= (1.0 - gain_x);
                P_[1][1] *= (1.0 - gain_x); // misma incertidumbre en x,y
                P_[2][2] *= (1.0 - gain_theta);
            }
            break;
    }
}

void EKF::normalize_angle(double& angle) const {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
}

double EKF::angle_diff(double a, double b) const {
    double diff = a - b;
    normalize_angle(diff);
    return diff;
}

} // namespace aon