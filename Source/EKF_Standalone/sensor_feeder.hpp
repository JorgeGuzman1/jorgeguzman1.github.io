#pragma once

#include "EKF.hpp"
#include <chrono>

namespace aon {

// Flags de configuración de sensores
constexpr bool UsingGPS = true;
constexpr bool UsingIMU = true;
constexpr bool UsingEncoders = true;

// Configuración de conversión (ajustar según tu robot)
constexpr double WHEEL_RADIUS_IN = 2.0;     // Radio de rueda (in) - 2" = 5.08cm
constexpr double GEAR_RATIO = 1.0;            // Relación de engranajes
constexpr double TICKS_PER_REV = 36000.0;      // Ticks por revolución (Rotation sensor)
constexpr double ENCODER_S_VAR = 0.01;        // Varianza encoder lineal (in²)
constexpr double ENCODER_M_VAR = 0.001;        // Varianza encoder angular (rad²)
constexpr double GYRO_VAR = 0.01;              // Varianza giroscopio (rad²/s²)
constexpr double GPS_VAR_X = 1.0;             // Varianza GPS en X (in²)
constexpr double GPS_VAR_Y = 1.0;             // Varianza GPS en Y (in²)
constexpr double GPS_VAR_HEADING = 0.1;        // Varianza GPS heading (rad²)

/**
 * @brief Datos simulados de sensores para testing
 */
struct SimulatedSensorData {
    double encoderS_vel = 0.0;     // Velocidad encoder lineal (RPM)
    double encoderM_angle = 0.0;   // Ángulo encoder angular (grados)
    double imu_yaw_rate = 0.0;     // Tasa yaw IMU (rad/s)
    double gps_x = 0.0;            // Posición GPS X (mm)
    double gps_y = 0.0;            // Posición GPS Y (mm)
    double gps_heading = 0.0;      // Heading GPS (grados)
    bool encoderS_ok = true;
    bool encoderM_ok = true;
    bool imu_ok = true;
    bool gps_ok = true;
};

/**
 * @brief Inicializa el feeder. Llamar una vez al iniciar el robot.
 */
void sensor_feeder_init();

/**
 * @brief Convierte ticks de Rotation a pulgadas lineales
 */
double rotation_to_linear_in(double ticks_rev);

/**
 * @brief Convierte ticks de Rotation a radianes
 */
double rotation_to_rad(double ticks_deg);

/**
 * @brief Actualiza el EKF con datos simulados. Llamar periódicamente (ej: cada 20ms)
 */
void feed_ekf_sensors_simulated(EKF& ekf, const SimulatedSensorData& sensor_data);

/**
 * @brief Genera datos simulados para una trayectoria específica
 */
SimulatedSensorData generate_sensor_data(double time_s);

/**
 * @brief Verifica la salud de los sensores
 * @return String con estado: "OK", "WARN", o "ERROR"
 */
const char* check_sensor_health();

/**
 * @brief Obtiene estado detallado de sensores para debug
 */
struct SensorStatus {
    bool encoderS_ok = false;
    bool encoderM_ok = false;
    bool imu_ok = false;
    bool gps_ok = false;
    double encoderS_vel = 0.0;
    double encoderM_angle = 0.0;
    double imu_yaw_rate = 0.0;
    double gps_x = 0.0;
    double gps_y = 0.0;
    double gps_heading = 0.0;
};

SensorStatus get_sensor_status(const SimulatedSensorData& sensor_data);

} // namespace aon