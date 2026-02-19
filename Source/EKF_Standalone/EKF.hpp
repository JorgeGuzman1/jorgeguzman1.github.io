#pragma once

#include <cstdint>

namespace aon {

/**
 * @brief Estado del EKF: pose y velocidades del robot
 */
struct EKFState {
    double x;        ///< Posición X en inches
    double y;        ///< Posición Y en inches
    double theta;    ///< Orientación (yaw) en radianes
    double v;        ///< Velocidad lineal in/s
    double omega;    ///< Velocidad angular en rad/s
};

/**
 * @brief Filtro de Kalman Extendido para localización 2D de robot móvil
 *
 * Implementa un EKF discreto con:
 * - Estado: [x, y, theta, v, omega]
 * - Predicción por odometría (encoderS + encoderM/IMU)
 * - Correcciones opcionales: GPS, IMU, encoders
 *
 * Uso típico:
 * \code
 * aon::EKF ekf;
 * ekf.predict_from_odometry(v_inps, omega_rad_s, dt_s);
 * if (gps_available) ekf.correct_with_gps(x_in, y_in, var_x, var_y);
 * aon::EKFState pose = ekf.get_state();
 * \endcode
 */
class EKF {
public:
    /**
     * @brief Constructor. Inicializa covarianza y ruido por defecto.
     */
    EKF();

    /**
     * @brief Predicción del estado por odometría (modelo cinemático)
     * @param v_inps Velocidad lineal medida (in/s)
     * @param omega_rad_s Velocidad angular medida (rad/s)
     * @param dt_s Intervalo de tiempo (s)
     */
    void predict_from_odometry(double v_inps, double omega_rad_s, double dt_s);

    /**
     * @brief Corrección con tasa de giro del giroscopio
     * @param yaw_rate_rad_s Tasa de yaw medida (rad/s)
     * @param var_yaw_rate Varianza de la medición (rad²/s²)
     */
    void correct_with_gyro(double yaw_rate_rad_s, double var_yaw_rate);

    /**
     * @brief Corrección con desplazamiento y rotación de encoders
     * @param ds_in Desplazamiento lineal delta (in)
     * @param dtheta_rad Rotación angular delta (rad)
     * @param var_ds Varianza del desplazamiento (in²)
     * @param var_dtheta Varianza de la rotación (rad²)
     */
    void correct_with_encoders(double ds_in, double dtheta_rad, double var_ds, double var_dtheta);

    /**
     * @brief Corrección con pose absoluta del GPS
     * @param x_in Posición X del GPS (in)
     * @param y_in Posición Y del GPS (in)
     * @param var_x Varianza en X (in²)
     * @param var_y Varianza en Y (in²)
     * @param use_heading Si true, también corrige heading con GPS
     * @param heading_deg Heading del GPS (grados) si use_heading=true
     * @param var_heading Varianza del heading (rad²) si use_heading=true
     */
    void correct_with_gps(double x_in, double y_in, double var_x, double var_y,
                         bool use_heading = false, double heading_deg = 0.0,
                         double var_heading = 0.0);

    /**
     * @brief Obtiene el estado estimado actual
     * @return Estado EKF (pose + velocidades)
     */
    EKFState get_state() const;

    /**
     * @brief Distancia total acumulada (suma de |ds|)
     * @return Distancia recorrida en inches
     */
    double get_distance_traveled_in() const;

private:
    // Estado y covarianza
    EKFState state_;
    double P_[5][5];  // Matriz de covarianza 5x5

    // Ruido del proceso
    double q_pos_;    // Varianza ruido posición (in²)
    double q_theta_;  // Varianza ruido heading (rad²)
    double q_v_;      // Varianza ruido velocidad (in²/s²)
    double q_omega_;  // Varianza ruido velocidad angular (rad²/s²)

    // Métricas
    double distance_traveled_in_;

    // Métodos internos
    void predict_covariance(double v, double omega, double dt);
    void update_covariance_gps(const double* z, const double* R, bool use_heading);
    void update_covariance_sensor(const double* z, const double* R, int sensor_type);
    void normalize_angle(double& angle) const;
    double angle_diff(double a, double b) const;
};

} // namespace aon