#include "core.hpp"

class Teleop_Mining : public rclcpp::Node{
public:
    Teleop_Mining() : Node("Teleop_Mining")    
    #ifdef HARDWARE_ENABLED
    , m_actuator_left("can0", 5)
    , m_actuator_right("can0",6)
    #endif
    {
        init_params();
        
        #ifdef HARDWARE_ENABLED
        config_motor(m_actuator_left);
        config_motor(m_actuator_right);
        #endif
        
        velocity_subscriber = create_subscription<Twist>("cmd_vel", 10, std::bind(&Teleop_Mining::callback_cmd_vel, this, std::placeholders::_1) );
        joy_sub = create_subscription<Joy>("joy", 10, std::bind(&Teleop_Mining::callback_joy, this, std::placeholders::_1) );
        timer = create_wall_timer(5s, std::bind(&Teleop_Mining::callback_motor_heartbeat, this)); 

    }
    
private:
    ROBOTSTATE_t robot_state;
    ROBOT_ACTUATION_t robot_actuation;
    XBOX_JOYSTICK_INPUT_t xbox_input;
    ROBOT_LIMITS_t robot_dimensions;
    
    TwistSubscription velocity_subscriber;
    JoySubscription joy_sub;
    rclcpp::TimerBase::SharedPtr timer;

    #ifdef HARDWARE_ENABLED
    SparkMax m_actuator_left;
    SparkMax m_actuator_right;
    #endif

    void init_params(){
        robot_state.XBOX = get_parameter("XBOX").as_bool();
        robot_state.robot_disabled = get_parameter("robot_disabled").as_bool();
        robot_state.manual_enabled = get_parameter("manual_enabled").as_bool();
        robot_state.PS4 = get_parameter("PS4").as_bool();
        robot_state.outdoor_mode = get_parameter("outdoor_mode").as_bool();
    }

    void callback_cmd_vel(){}
    void callback_joy(){}
    void control_robot(){
        
    }

    #ifdef HARDWARE_ENALBED
    void config_motor(SparkMax& motor){
        motor.SetIdleMode(IdleMode::kBrake);
        motor.SetMotorType(MotorType::kBrushless);
        motor.BurnFlash();
    }
    #endif

    void callback_motor_heartbeat(){
        #ifdef HARDWARE_ENABLED   
        try{
            m_actuator_left.Heartbeat();
            m_actuator.right.Heartbeat();
        }catch(const std::exception &e){
            std::cerr << "Motor Heartbeat Error: " << e.what() << std::endl;
        }
        #endif
   
    };

};



int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Teleop_Mining>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}