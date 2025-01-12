#include "DiggingBelt.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DiggingBelt>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

DiggingBelt::DiggingBelt()
    : Node("digging_belt")
    , m_belt1("can0", BELT_1_CAN_ID)
    , m_belt2("can0", BELT_2_CAN_ID)
{
    configureBelts();
    configurePID();
    
    velocity_publisher = create_publisher<std_msgs::msg::Float64>("belt/velocity", 10);
    temperature_publisher = create_publisher<std_msgs::msg::Float64>("belt/temperature", 10);
    timer_ = create_wall_timer(100ms, std::bind(&DiggingBelt::periodic, this));
    speed_subscriber = create_subscription<std_msgs::msg::Float64>("mining/belt_speed", 10,std::bind(&DiggingBelt::handleSpeedCommand, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "Digging Belt initialized");
}

Float64Subscriber speed_subscriber;
/**
 * @brief Sets Mining Belt Speed
 * @NOTE runBelt() setBeltSpeed()? Check JAVA FRC Code
 * @param msg 
 */
void handleSpeedCommand(const std_msgs::msg::Float64::SharedPtr msg) {
    setBeltSpeed(msg->data);  // Or use runBelt() depending on your needs
}

void DiggingBelt::configureBelts() {
    // Configure follower
    m_belt2.SetFollowerID(BELT_1_CAN_ID);
    m_belt2.SetFollowerConfig(1);  // Basic follower mode
    m_belt2.SetInverted(true);  // Invert follower

    // Configure motor settings
    m_belt1.SetMotorType(MotorType::kBrushless);
    m_belt2.SetMotorType(MotorType::kBrushless);
    
    // Burn configuration
    m_belt1.BurnFlash();
    m_belt2.BurnFlash();
}

/**
 * @brief Set PID values
 * @note Grayson says PID coming soon. Hopefully asap
 */
void DiggingBelt::configurePID() {
    p_belt = std::make_unique<PIDController>(m_belt1);
    
    // Configure PID settings
    p_belt->SetP(0, BELT_kP);
    p_belt->SetI(0, BELT_kI);
    p_belt->SetD(0, BELT_kD);
    p_belt->SetIZone(0, BELT_kIZ);
    p_belt->SetF(0, BELT_kFF);
}

/**
 * @brief Periodically prints sensor data
 */
void DiggingBelt::periodic() {
    reportSensors();
}

void DiggingBelt::setBeltSpeed(double speed) {
    m_belt1.SetDutyCycle(speed);
    belt_running = true;
    RCLCPP_INFO(get_logger(), "Belt speed set to: %f", speed);
}

void DiggingBelt::runBelt(bool reverse) {
    double target_speed = reverse ? -belt_spee : belt_speed_;
    p_belt->SetReference(target_speed, CtrlType::kVelocity);
    belt_running = true;
    RCLCPP_INFO(get_logger(), "Belt running at speed: %f", target_speed);
}

/**
 * @brief Stops belt via duty cycle
 */
void DiggingBelt::stopBelt() {
    m_belt1.SetDutyCycle(0.0);
    belt_running = false;
    RCLCPP_INFO(get_logger(), "Belt stopped");
}

/**
 * @brief Logging function 
*/
void DiggingBelt::reportSensors() {
    auto velocity_msg = std_msgs::msg::Float64();
    velocity_msg.data = m_belt1.GetVelocity();
    velocity_publisher->publish(velocity_msg);

    auto temp_msg = std_msgs::msg::Float64();
    temp_msg.data = m_belt1.GetTemperature();
    temperature_publisher->publish(temp_msg);
    RCLCPP_DEBUG(get_logger(), "Belt Velocity: %f, Temperature: %f", velocity_msg.data, temp_msg.data);
}