#include "ctre/phoenix6/TalonFX.hpp"
#include "RobotBase.hpp"
#include "Joystick.hpp"
#include <fstream>
#include <chrono>
#include <iostream>
#include <csignal>
#include <thread>
#include <string>
#include <limits>
#include <units/angle.h>
std::ofstream rpm_file("rpm_data.csv", std::ios::trunc);
std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();

void signalHandler(int signum) {
    system("python3 graph.py");

    // cleanup and close up stuff here  
    // terminate program  
    exit(signum);  
}

using namespace ctre::phoenix6;

/**
 * This is the main robot. Put all actuators, sensors,
 * game controllers, etc. in this class.
 */
class Robot : public RobotBase {
private:
    /* This can be a CANivore name, CANivore serial number,
     * SocketCAN interface, or "*" to select any CANivore. */
    static constexpr char const *CANBUS_NAME = "can0";

    /* devices */
    hardware::TalonFX m_motor{15, CANBUS_NAME};

    /* control requests */
    controls::VelocityDutyCycle m_velocityDutyCycle{0_tps};

    // robot init, set slot 0 gains
    configs::Slot0Configs slot0Configs;

    // Store RPM value for later use
    double input_rpm = 0.0;
    double current_rpm = 0.0; // Track the current RPM
    double ramp_rate = 50.0; // Ramp rate in RPM per second

public:
    /* main robot interface */
    void RobotInit() override;
    void RobotPeriodic() override;

    bool IsEnabled() override;
    void EnabledInit() override;
    void EnabledPeriodic() override;

    void DisabledInit() override;
    void DisabledPeriodic() override;
};

/**
 * Runs once at code initialization.
 */
void Robot::RobotInit() {
    // Set Slot 0 configurations
    slot0Configs.kV = 0.008;
    slot0Configs.kP = 0.02;
    slot0Configs.kI = 0.04;
    slot0Configs.kD = 0.00;

    m_motor.GetConfigurator().Apply(slot0Configs, 50_ms);
}

/**
 * Runs periodically during program execution.
 */
void Robot::RobotPeriodic() {
}

/**
 * Returns whether the robot should be enabled.
 */
bool Robot::IsEnabled() {
    return true;
}

/**
 * Runs when transitioning from disabled to enabled.
 */
void Robot::EnabledInit() {
    // Ask user for RPM input once when enabled
    std::cout << "Enter desired RPM: ";
    while (!(std::cin >> input_rpm)) {
        std::cin.clear(); // clear input buffer
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // ignore invalid input
        std::cout << "Invalid input, please enter a numeric value for RPM: ";
    }
}

/**
 * Runs periodically while enabled.
 */
void Robot::EnabledPeriodic() {
    signal(SIGINT, signalHandler);
    
    double RPS = input_rpm / 60; // Target RPM converted to RPS (rotations per second)
    double target_rpm = input_rpm;
    
    // Ramp RPM to target
    if (current_rpm < target_rpm) {
        current_rpm += ramp_rate * 0.2; // Increment RPM towards target
        if (current_rpm > target_rpm) {
            current_rpm = target_rpm; // Cap to the target RPM
        }
    } else if (current_rpm > target_rpm) {
        current_rpm -= ramp_rate * 0.02; // Decrement RPM towards target
        if (current_rpm < target_rpm) {
            current_rpm = target_rpm; // Cap to the target RPM
        }
    }

    // Set motor RPM
    RPS = current_rpm / 60;
    m_velocityDutyCycle.Slot = 0;
    m_motor.SetControl(m_velocityDutyCycle.WithVelocity(units::angular_velocity::turns_per_second_t(RPS)));

    auto signal = m_motor.GetVelocity();
    auto motor_RPS = signal.GetValue();
    double motor_RPM = (motor_RPS.to<double>() * 60.0);
    auto elapsed_time = std::chrono::steady_clock::now() - start_time;
    auto seconds = std::chrono::duration_cast<std::chrono::duration<double>>(elapsed_time).count();

    rpm_file << seconds << "," << motor_RPM << std::endl;

    std::cout << "\033[2J\033[H" << "Encoder RPM: " << motor_RPM << " at " << seconds << " seconds" << std::flush;
}


/**
 * Runs when transitioning from enabled to disabled,
 * including after robot startup.
 */
void Robot::DisabledInit() {}

/**
 * Runs periodically while disabled.
 */
void Robot::DisabledPeriodic() {
    m_motor.SetControl(controls::NeutralOut{});
}

/* ------ main function ------ */
int main() {
    /* create and run robot */
    Robot robot{};
    // robot.SetLoopTime(20_ms); // optionally change loop time for periodic calls
    return robot.Run();
}

