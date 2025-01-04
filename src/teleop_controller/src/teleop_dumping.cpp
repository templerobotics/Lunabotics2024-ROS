#include "core.hpp"

//Adapt to actual sparkmaxes/motors needed for "dumping"

class Teleop_Dumping : public rclcpp::Node{
public:
    Teleop_Dumping() : Node("Teleop_dumping")    
    #ifdef HARDWARE_ENABLED
    , m_left_front("can0", 7)
    , m_left_rear("can0", 8)
    , m_right_front("can0", 9)
    , m_right_rear("can0", 10)
    , m_right_rear("can0", 11)
    , m_right_rear("can0", 12)
    #endif
    {
        init_params();
        velocity_subscriber = create_subscription<Twist>("cmd_vel", 10, std::bind(&Teleop_Dumping::callback_cmd_vel, this, std::placeholders::_1) );
        joy_sub = create_subscription<Joy>("joy", 10, std::bind(&Teleop_Dumping::callback_joy, this, std::placeholders::_1) );
        timer = create_wall_timer(5s, std::bind(&Teleop_Dumping:callback_motor_heartbeat, this)); 

    }
    
private:
    ROBOTSTATE_t robot_state;
    ROBOT_ACTUATION_t robot_actuation;
    XBOX_JOYSTICK_INPUT_t xbox_input;
    ROBOT_LIMITS_t robot_dimensions;
    TwistSubscription velocity_subscriber;
    JoySubscription joy_sub;
    rclcpp::TimerBase::SharedPtr timer;


    void init_params(){
        robot_state.XBOX = get_parameter("XBOX").as_bool();
        robot_state.robot_disabled = get_parameter("robot_disabled").as_bool();
        robot_state.manual_enabled = get_parameter("manual_enabled").as_bool();
        robot_state.PS4 = get_parameter("PS4").as_bool();
        robot_state.outdoor_mode = get_parameter("outdoor_mode").as_bool();
    }

    void callback_cmd_vel(){}
    void callback_joy(){}
    void control_robot(){}
    //Adapt to actual sparkmaxes/motors needed for "dumping"
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
    auto node = std::make_shared<Teleop_Dumping>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}