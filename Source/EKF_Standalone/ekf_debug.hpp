#pragma once

#include "EKF.hpp"
#include "sensor_feeder.hpp"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <cstring>
#include <cmath>
#include <chrono>

namespace aon {

// Configuración de debug
constexpr bool EKF_PASSIVE_MODE = true;
constexpr uint32_t DEBUG_PRINT_INTERVAL_MS = 150;  // Imprimir cada 150ms
constexpr int PRECISION_DIGITS = 3;

// Variables estáticas para debug
static uint64_t last_debug_print = 0;
static uint32_t debug_update_count = 0;

/**
 * @brief Formatea un double con precisión fija
 */
static std::string format_double(double value, int precision = PRECISION_DIGITS) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(precision) << value;
    return oss.str();
}

/**
 * @brief Formatea ángulo en grados
 */
static std::string format_angle_deg(double radians) {
    double degrees = radians * 180.0 / M_PI;
    return format_double(degrees) + "°";
}

/**
 * @brief Formatea salud de sensores en string corto
 */
static std::string format_sensor_health(const SensorStatus& status) {
    std::string health = "";

    if (UsingEncoders) {
        health += status.encoderS_ok ? "S" : "s";
        health += status.encoderM_ok ? "M" : "m";
    }

    if (UsingIMU) {
        health += status.imu_ok ? "I" : "i";
    }

    if (UsingGPS) {
        health += status.gps_ok ? "G" : "g";
    }

    return health.empty() ? "NONE" : health;
}

/**
 * @brief Imprime línea compacta de estado del EKF
 */
void debug_print_ekf_state(const EKF& ekf, const SensorStatus& sensor_status);

/**
 * @brief Imprime estado detallado de sensores
 */
void debug_print_sensor_status(const SensorStatus& status);

/**
 * @brief Imprime métricas de rendimiento del EKF
 */
void debug_print_performance(const EKF& ekf);

/**
 * @brief Verificación rápida de salud del sistema
 */
bool debug_system_healthy(const SensorStatus& status);

/**
 * @brief Reinicia contadores de debug
 */
void debug_reset_counters();

/**
 * @brief Imprime resumen de configuración
 */
void debug_print_config();

/**
 * @brief Función principal de debug - llamar periódicamente
 */
void debug_update(const EKF& ekf, const SensorStatus& sensor_status);

} // namespace aon