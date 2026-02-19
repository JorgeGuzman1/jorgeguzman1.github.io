#include "ekf_debug.hpp"

namespace aon {

/**
 * @brief Imprime línea compacta de estado del EKF
 */
void debug_print_ekf_state(const EKF& ekf, const SensorStatus& sensor_status) {
    if (!EKF_PASSIVE_MODE) return;

    auto now = std::chrono::steady_clock::now();
    auto duration = now.time_since_epoch();
    uint64_t current_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();

    if (current_time_ms - last_debug_print < DEBUG_PRINT_INTERVAL_MS) {
        return;
    }

    last_debug_print = current_time_ms;
    debug_update_count++;

    EKFState state = ekf.get_state();
    double distance = ekf.get_distance_traveled_in();
    std::string sensor_health = format_sensor_health(sensor_status);

    std::cout << "[EKF] Pose=(x=" << format_double(state.x) << "in, "
              << "y=" << format_double(state.y) << "in, "
              << "θ=" << format_angle_deg(state.theta) << ") "
              << "v=" << format_double(state.v) << "in/s  "
              << "dist=" << format_double(distance) << "in  "
              << "SENSORS=" << sensor_health
              << "  updates=" << debug_update_count << std::endl;
}

/**
 * @brief Imprime estado detallado de sensores
 */
void debug_print_sensor_status(const SensorStatus& status) {
    std::cout << "\n=== SENSOR STATUS ===" << std::endl;

    std::string health = check_sensor_health();
    std::cout << "Overall Health: " << health << std::endl;

    if (UsingEncoders) {
        std::cout << "EncoderS (linear): " << (status.encoderS_ok ? "OK" : "ERROR")
                  << " vel=" << format_double(status.encoderS_vel) << " RPM" << std::endl;
        std::cout << "EncoderM (angular): " << (status.encoderM_ok ? "OK" : "ERROR")
                  << " angle=" << format_double(status.encoderM_angle) << "°" << std::endl;
    }

    if (UsingIMU) {
        std::cout << "IMU (gyro): " << (status.imu_ok ? "OK" : "ERROR")
                  << " rate=" << format_double(status.imu_yaw_rate) << " rad/s" << std::endl;
    }

    if (UsingGPS) {
        std::cout << "GPS: " << (status.gps_ok ? "OK" : "ERROR")
                  << " pos=(" << format_double(status.gps_x/25.4) << "in, "
                  << format_double(status.gps_y/25.4) << "in, "
                  << format_double(status.gps_heading) << "°)" << std::endl;
    }

    std::cout << "===================" << std::endl;
}

/**
 * @brief Imprime métricas de rendimiento del EKF
 */
void debug_print_performance(const EKF& ekf) {
    EKFState state = ekf.get_state();
    double distance = ekf.get_distance_traveled_in();

    std::cout << "\n=== EKF PERFORMANCE ===" << std::endl;
    std::cout << "Current State:" << std::endl;
    std::cout << "  Position: (" << format_double(state.x) << "in, "
              << format_double(state.y) << "in)" << std::endl;
    std::cout << "  Heading: " << format_angle_deg(state.theta) << std::endl;
    std::cout << "  Linear Vel: " << format_double(state.v) << " in/s" << std::endl;
    std::cout << "  Angular Vel: " << format_double(state.omega) << " rad/s" << std::endl;
    std::cout << "  Distance Traveled: " << format_double(distance) << " in" << std::endl;
    std::cout << "  Debug Updates: " << debug_update_count << std::endl;
    std::cout << "=====================" << std::endl;
}

/**
 * @brief Verificación rápida de salud del sistema
 */
bool debug_system_healthy(const SensorStatus& status) {
    const char* health = check_sensor_health();
    bool healthy = (strcmp(health, "OK") == 0);

    if (!healthy && EKF_PASSIVE_MODE) {
        std::cout << "[EKF DEBUG] WARNING: System health degraded - " << health << std::endl;
    }

    return healthy;
}

/**
 * @brief Reinicia contadores de debug
 */
void debug_reset_counters() {
    debug_update_count = 0;
    auto now = std::chrono::steady_clock::now();
    auto duration = now.time_since_epoch();
    last_debug_print = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
    std::cout << "[EKF DEBUG] Counters reset" << std::endl;
}

/**
 * @brief Imprime resumen de configuración
 */
void debug_print_config() {
    std::cout << "\n=== EKF CONFIGURATION ===" << std::endl;
    std::cout << "Passive Mode: " << (EKF_PASSIVE_MODE ? "ENABLED" : "DISABLED") << std::endl;
    std::cout << "Print Interval: " << DEBUG_PRINT_INTERVAL_MS << " ms" << std::endl;
    std::cout << "Using GPS: " << (UsingGPS ? "YES" : "NO") << std::endl;
    std::cout << "Using IMU: " << (UsingIMU ? "YES" : "NO") << std::endl;
    std::cout << "Using Encoders: " << (UsingEncoders ? "YES" : "NO") << std::endl;
    std::cout << "Wheel Radius: " << format_double(WHEEL_RADIUS_IN) << " in" << std::endl;
    std::cout << "Gear Ratio: " << format_double(GEAR_RATIO) << std::endl;
    std::cout << "========================" << std::endl;
}

/**
 * @brief Función principal de debug - llamar periódicamente
 */
void debug_update(const EKF& ekf, const SensorStatus& sensor_status) {
    if (!EKF_PASSIVE_MODE) return;

    // Imprimir estado compacto periódicamente
    debug_print_ekf_state(ekf, sensor_status);

    // Verificación de salud cada ~2 segundos
    static uint64_t last_health_check = 0;
    auto now = std::chrono::steady_clock::now();
    auto duration = now.time_since_epoch();
    uint64_t current_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();

    if (current_time_ms - last_health_check > 2000) {
        debug_system_healthy(sensor_status);
        last_health_check = current_time_ms;
    }
}

} // namespace aon