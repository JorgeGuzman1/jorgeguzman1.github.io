#include "EKF.hpp"
#include "sensor_feeder.hpp"
#include "ekf_debug.hpp"
#include <iostream>
#include <thread>
#include <chrono>
#include <iomanip>

int main() {
    std::cout << "=== EKF STANDALONE TEST ===" << std::endl;
    std::cout << "Testing Extended Kalman Filter with simulated sensor data" << std::endl;
    std::cout << "========================================" << std::endl;

    // Inicializar sistema
    aon::sensor_feeder_init();

    // Mostrar configuración
    aon::debug_print_config();

    // Crear instancia del EKF
    aon::EKF ekf;

    // Variables para la simulación
    double start_time = 0.0;
    double current_time = 0.0;
    const double sim_duration = 15.0;  // Simular 15 segundos
    const double dt = 0.02;           // 50Hz (cada 20ms)

    std::cout << "\nStarting simulation..." << std::endl;
    std::cout << "Duration: " << sim_duration << " seconds" << std::endl;
    std::cout << "Update rate: " << (1.0/dt) << " Hz" << std::endl;
    std::cout << "Trajectory: Square movement (24in sides)" << std::endl;
    std::cout << "========================================" << std::endl;

    // Reiniciar contadores de debug
    aon::debug_reset_counters();

    // Ciclo principal de simulación
    int step_count = 0;
    while (current_time < sim_duration) {
        // Generar datos simulados para este timestep
        aon::SimulatedSensorData sensor_data = aon::generate_sensor_data(current_time);

        // Obtener estado de sensores para debug
        aon::SensorStatus status = aon::get_sensor_status(sensor_data);

        // Alimentar EKF con datos de sensores
        aon::feed_ekf_sensors_simulated(ekf, sensor_data);

        // Actualizar sistema de debug
        aon::debug_update(ekf, status);

        // Imprimir información detallada cada 1 segundo
        if (step_count % 50 == 0) {  // Cada 1 segundo (50 steps * 20ms)
            std::cout << "\n--- Time: " << std::fixed << std::setprecision(2)
                      << current_time << "s (Step " << step_count << ") ---" << std::endl;

            // Mostrar datos reales simulados
            std::cout << "GROUND TRUTH:" << std::endl;
            std::cout << "  GPS: (" << std::setprecision(1)
                      << sensor_data.gps_x/25.4 << "in, "
                      << sensor_data.gps_y/25.4 << "in, "
                      << sensor_data.gps_heading << "°)" << std::endl;
            std::cout << "  EncoderS: " << std::setprecision(1)
                      << sensor_data.encoderS_vel << " RPM" << std::endl;
            std::cout << "  IMU Rate: " << std::setprecision(3)
                      << sensor_data.imu_yaw_rate << " rad/s" << std::endl;

            // Mostrar estimación del EKF
            aon::EKFState ekf_state = ekf.get_state();
            std::cout << "EKF ESTIMATE:" << std::endl;
            std::cout << "  Position: (" << std::setprecision(1)
                      << ekf_state.x << "in, " << ekf_state.y << "in)" << std::endl;
            std::cout << "  Heading: " << std::setprecision(1)
                      << (ekf_state.theta * 180.0 / M_PI) << "°" << std::endl;
            std::cout << "  Velocity: " << std::setprecision(2)
                      << ekf_state.v << "in/s, " << ekf_state.omega << "rad/s" << std::endl;

            // Calcular error
            double ground_x = sensor_data.gps_x / 25.4;
            double ground_y = sensor_data.gps_y / 25.4;
            double ground_heading = sensor_data.gps_heading * M_PI / 180.0;

            double pos_error = std::sqrt(std::pow(ekf_state.x - ground_x, 2) +
                                       std::pow(ekf_state.y - ground_y, 2));
            double heading_error = std::abs(ekf_state.theta - ground_heading);
            while (heading_error > M_PI) heading_error -= 2.0 * M_PI;

            std::cout << "ERRORS:" << std::endl;
            std::cout << "  Position: " << std::setprecision(2) << pos_error << "in" << std::endl;
            std::cout << "  Heading: " << std::setprecision(1)
                      << (heading_error * 180.0 / M_PI) << "°" << std::endl;
        }

        // Esperar el timestep
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(dt * 1000)));

        current_time += dt;
        step_count++;
    }

    // Resultados finales
    std::cout << "\n=== SIMULATION COMPLETE ===" << std::endl;
    aon::debug_print_performance(ekf);

    // Estado final vs ground truth
    aon::SimulatedSensorData final_sensor = aon::generate_sensor_data(sim_duration);
    aon::EKFState final_state = ekf.get_state();

    std::cout << "\nFINAL COMPARISON:" << std::endl;
    std::cout << "Ground Truth: (" << std::fixed << std::setprecision(1)
              << final_sensor.gps_x/25.4 << "in, "
              << final_sensor.gps_y/25.4 << "in, "
              << final_sensor.gps_heading << "°)" << std::endl;
    std::cout << "EKF Estimate: (" << std::setprecision(1)
              << final_state.x << "in, " << final_state.y << "in, "
              << (final_state.theta * 180.0 / M_PI) << "°)" << std::endl;

    double final_error = std::sqrt(std::pow(final_state.x - final_sensor.gps_x/25.4, 2) +
                                 std::pow(final_state.y - final_sensor.gps_y/25.4, 2));
    std::cout << "Final Position Error: " << std::setprecision(2) << final_error << "in" << std::endl;
    std::cout << "Total Distance Traveled: " << std::setprecision(1)
              << ekf.get_distance_traveled_in() << "in" << std::endl;

    std::cout << "\nTest completed successfully!" << std::endl;

    return 0;
}