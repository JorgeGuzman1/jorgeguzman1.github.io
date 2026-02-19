#include "sensor_feeder.hpp"
#include <cmath>
#include <thread>
#include <chrono>

namespace aon {

// Variables para simulación
static double prev_encoderS_pos = 0.0;
static double prev_encoderM_angle = 0.0;
static uint64_t prev_update_time_ms = 0;

/**
 * @brief Inicializa el feeder. Llamar una vez al iniciar el robot.
 */
void sensor_feeder_init() {
    prev_encoderS_pos = 0.0;
    prev_encoderM_angle = 0.0;

    auto now = std::chrono::steady_clock::now();
    auto duration = now.time_since_epoch();
    prev_update_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
}

/**
 * @brief Convierte ticks de Rotation a pulgadas lineales
 */
double rotation_to_linear_in(double ticks_rev) {
    double revs = ticks_rev / GEAR_RATIO;
    return revs * 2.0 * M_PI * WHEEL_RADIUS_IN;
}

/**
 * @brief Convierte ticks de Rotation a radianes
 */
double rotation_to_rad(double ticks_deg) {
    return (ticks_deg / GEAR_RATIO) * M_PI / 180.0;
}

/**
 * @brief Genera datos simulados para una trayectoria específica
 * Simula un robot moviéndose en un cuadrado
 */
SimulatedSensorData generate_sensor_data(double time_s) {
    SimulatedSensorData data;

    // Parámetros de la trayectoria
    const double side_length_in = 24.0;  // Lado del cuadrado: 24 pulgadas
    const double speed_inps = 12.0;       // Velocidad: 12 in/s
    const double turn_rate_rads = M_PI/2; // 90°/s

    const double side_time = side_length_in / speed_inps;  // Tiempo por lado: 2s
    const double turn_time = (M_PI/2) / turn_rate_rads;     // Tiempo por giro: 1s
    const double cycle_time = 4 * side_time + 4 * turn_time; // Ciclo completo: 12s

    // Calcular posición en el ciclo
    double cycle_progress = fmod(time_s, cycle_time);

    if (cycle_progress < side_time) {
        // Lado 1: mover hacia adelante (+X)
        double progress = cycle_progress / side_time;
        data.encoderS_vel = 60.0;  // 60 RPM constante
        data.encoderM_angle = 0.0; // Sin rotación
        data.imu_yaw_rate = 0.0;
        data.gps_x = progress * side_length_in * 25.4;  // Convertir a mm
        data.gps_y = 0.0;
        data.gps_heading = 0.0;
    } else if (cycle_progress < side_time + turn_time) {
        // Giro 1: girar 90° izquierda
        double progress = (cycle_progress - side_time) / turn_time;
        data.encoderS_vel = 0.0;
        data.encoderM_angle = progress * 90.0;  // 0° a 90°
        data.imu_yaw_rate = turn_rate_rads;
        data.gps_x = side_length_in * 25.4;
        data.gps_y = 0.0;
        data.gps_heading = progress * 90.0;
    } else if (cycle_progress < 2 * side_time + turn_time) {
        // Lado 2: mover hacia la izquierda (+Y)
        double progress = (cycle_progress - side_time - turn_time) / side_time;
        data.encoderS_vel = 60.0;
        data.encoderM_angle = 90.0;  // Mantener 90°
        data.imu_yaw_rate = 0.0;
        data.gps_x = side_length_in * 25.4;
        data.gps_y = progress * side_length_in * 25.4;
        data.gps_heading = 90.0;
    } else if (cycle_progress < 2 * side_time + 2 * turn_time) {
        // Giro 2: girar 90° izquierda
        double progress = (cycle_progress - 2 * side_time - turn_time) / turn_time;
        data.encoderS_vel = 0.0;
        data.encoderM_angle = 90.0 + progress * 90.0;  // 90° a 180°
        data.imu_yaw_rate = turn_rate_rads;
        data.gps_x = side_length_in * 25.4;
        data.gps_y = side_length_in * 25.4;
        data.gps_heading = 90.0 + progress * 90.0;
    } else if (cycle_progress < 3 * side_time + 2 * turn_time) {
        // Lado 3: mover hacia atrás (-X)
        double progress = (cycle_progress - 2 * side_time - 2 * turn_time) / side_time;
        data.encoderS_vel = 60.0;
        data.encoderM_angle = 180.0;  // Mantener 180°
        data.imu_yaw_rate = 0.0;
        data.gps_x = (1.0 - progress) * side_length_in * 25.4;
        data.gps_y = side_length_in * 25.4;
        data.gps_heading = 180.0;
    } else if (cycle_progress < 3 * side_time + 3 * turn_time) {
        // Giro 3: girar 90° izquierda
        double progress = (cycle_progress - 3 * side_time - 2 * turn_time) / turn_time;
        data.encoderS_vel = 0.0;
        data.encoderM_angle = 180.0 + progress * 90.0;  // 180° a 270°
        data.imu_yaw_rate = turn_rate_rads;
        data.gps_x = 0.0;
        data.gps_y = side_length_in * 25.4;
        data.gps_heading = 180.0 + progress * 90.0;
    } else if (cycle_progress < 4 * side_time + 3 * turn_time) {
        // Lado 4: mover hacia la derecha (-Y)
        double progress = (cycle_progress - 3 * side_time - 3 * turn_time) / side_time;
        data.encoderS_vel = 60.0;
        data.encoderM_angle = 270.0;  // Mantener 270°
        data.imu_yaw_rate = 0.0;
        data.gps_x = 0.0;
        data.gps_y = (1.0 - progress) * side_length_in * 25.4;
        data.gps_heading = 270.0;
    } else {
        // Giro 4: girar 90° izquierda para volver al inicio
        double progress = (cycle_progress - 4 * side_time - 3 * turn_time) / turn_time;
        data.encoderS_vel = 0.0;
        data.encoderM_angle = 270.0 + progress * 90.0;  // 270° a 360° (0°)
        data.imu_yaw_rate = turn_rate_rads;
        data.gps_x = 0.0;
        data.gps_y = 0.0;
        data.gps_heading = 270.0 + progress * 90.0;
        if (data.gps_heading >= 360.0) data.gps_heading -= 360.0;
    }

    // Agregar ruido realista
    data.encoderS_vel += (rand() % 200 - 100) / 100.0;  // ±1 RPM de ruido
    data.encoderM_angle += (rand() % 20 - 10) / 10.0;   // ±1° de ruido
    data.imu_yaw_rate += (rand() % 100 - 50) / 10000.0; // ±0.005 rad/s de ruido
    data.gps_x += (rand() % 100 - 50) / 10.0;          // ±5mm de ruido
    data.gps_y += (rand() % 100 - 50) / 10.0;          // ±5mm de ruido
    data.gps_heading += (rand() % 20 - 10) / 10.0;     // ±1° de ruido

    return data;
}

/**
 * @brief Actualiza el EKF con datos de sensores simulados
 */
void feed_ekf_sensors_simulated(EKF& ekf, const SimulatedSensorData& sensor_data) {
    // Obtener tiempo actual
    auto now = std::chrono::steady_clock::now();
    auto duration = now.time_since_epoch();
    uint64_t current_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();

    double dt_s = (current_time_ms - prev_update_time_ms) / 1000.0;

    if (dt_s <= 0.0 || dt_s > 0.1) {
        prev_update_time_ms = current_time_ms;
        return; // Ignorar dt inválidos
    }

    // Variables para mediciones
    double v_inps = 0.0;  // Velocity in inches/sec
    double omega_rad_s = 0.0;
    bool have_v = false;
    bool have_omega = false;

    // 1. Obtener velocidad lineal de encoderS
    if (UsingEncoders && sensor_data.encoderS_ok) {
        double rev_per_min = sensor_data.encoderS_vel; // RPM
        double rev_per_sec = rev_per_min / 60.0;
        v_inps = rotation_to_linear_in(rev_per_sec); // Now in inches/sec
        have_v = true;
    }

    // 2. Obtener rotación de encoderM o IMU
    if (UsingEncoders && sensor_data.encoderM_ok) {
        double dangle_deg = sensor_data.encoderM_angle - prev_encoderM_angle;
        double dangle_rad = rotation_to_rad(dangle_deg);
        omega_rad_s = dangle_rad / dt_s;
        have_omega = true;
        prev_encoderM_angle = sensor_data.encoderM_angle;
    } else if (UsingIMU && sensor_data.imu_ok) {
        omega_rad_s = sensor_data.imu_yaw_rate; // usar eje Z para yaw
        have_omega = true;
    }

    // 3. Predicción del EKF con odometría
    if (have_v && have_omega) {
        ekf.predict_from_odometry(v_inps, omega_rad_s, dt_s);
    }

    // 4. Correcciones sensoriales

    // Corrección con giroscopio (si disponible)
    if (UsingIMU && sensor_data.imu_ok) {
        ekf.correct_with_gyro(sensor_data.imu_yaw_rate, GYRO_VAR);
    }

    // Corrección con encoders (delta measurements)
    if (UsingEncoders && sensor_data.encoderS_ok && sensor_data.encoderM_ok) {
        double ds_rev = (sensor_data.encoderS_vel / 60.0) * dt_s; // RPM * dt_s = rev
        double ds_in = rotation_to_linear_in(ds_rev);

        double dtheta_deg = (sensor_data.encoderM_angle - prev_encoderM_angle);
        double dtheta_rad = rotation_to_rad(dtheta_deg);

        ekf.correct_with_encoders(ds_in, dtheta_rad, ENCODER_S_VAR, ENCODER_M_VAR);
    }

    // Corrección con GPS (si disponible)
    if (UsingGPS && sensor_data.gps_ok) {
        double x_in = sensor_data.gps_x / 25.4; // mm -> inches (1in = 25.4mm)
        double y_in = sensor_data.gps_y / 25.4; // mm -> inches

        // Usar heading del GPS si está disponible
        bool use_heading = true;
        double heading_deg = sensor_data.gps_heading;

        ekf.correct_with_gps(x_in, y_in, GPS_VAR_X, GPS_VAR_Y,
                            use_heading, heading_deg, GPS_VAR_HEADING);
    }

    prev_update_time_ms = current_time_ms;
}

/**
 * @brief Verifica la salud de los sensores
 */
const char* check_sensor_health() {
    return "OK"; // En simulación siempre OK
}

SensorStatus get_sensor_status(const SimulatedSensorData& sensor_data) {
    SensorStatus status;

    status.encoderS_ok = sensor_data.encoderS_ok;
    status.encoderM_ok = sensor_data.encoderM_ok;
    status.imu_ok = sensor_data.imu_ok;
    status.gps_ok = sensor_data.gps_ok;

    status.encoderS_vel = sensor_data.encoderS_vel;
    status.encoderM_angle = sensor_data.encoderM_angle;
    status.imu_yaw_rate = sensor_data.imu_yaw_rate;
    status.gps_x = sensor_data.gps_x;
    status.gps_y = sensor_data.gps_y;
    status.gps_heading = sensor_data.gps_heading;

    return status;
}

} // namespace aon