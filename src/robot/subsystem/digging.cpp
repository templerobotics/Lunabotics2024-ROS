#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include <geometry_msgs/msg/twist.hpp>

#include "SparkMax.hpp"


class digging : public rclcpp::Node {
  public:
    bool topLimitLeft = false, topLimitRight = false;
    digging() : Node("digging"),
    leadscrew_left("can0", 9),
    leadscrew_right("can0", 8),
    actuator_left("can0", 7),
    actuator_right("can0", 6),
    digging_left("can0", 11),
    digging_right("can0", 10)
    {
        cmd_state_pub = create_publisher<std_msgs::msg::Bool>("reset", 10);
        timer_diagnostics = create_wall_timer(std::chrono::milliseconds(200), std::bind(&digging::periodic, this));
        initMotors();
        timer_diagnostics = create_wall_timer(std::chrono::milliseconds(200), std::bind(&digging::periodic, this));
        leadscrew_velocity_subscriber = create_subscription<std_msgs::msg::Float64>(
        "/cmd_vel_leadscrew", 10, std::bind(&digging::velocity_callback_leadscrew, this, std::placeholders::_1));
        actuator_velocity_subscriber = create_subscription<std_msgs::msg::Float64>(
        "/cmd_vel_actuator", 10, std::bind(&digging::velocity_callback_actuator, this, std::placeholders::_1));
        digging_velocity_subscriber = create_subscription<std_msgs::msg::Float64>(
        "/cmd_vel_digging", 10, std::bind(&digging::velocity_callback_digging, this, std::placeholders::_1));
        cmd_vel_drivebase_pub = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        cmd_vel_leadscrew_pub = create_publisher<std_msgs::msg::Float64>("cmd_vel_leadscrew", 10);
        cmd_vel_actuator_pub = create_publisher<std_msgs::msg::Float64>("cmd_vel_actuator", 10);
        cmd_vel_digging_pub = create_publisher<std_msgs::msg::Float64>("cmd_vel_digging", 10);
        cmd_vel_dumping_pub = create_publisher<std_msgs::msg::Float64>("cmd_vel_dumping", 10);
        cmd_state_subscriber = create_subscription<std_msgs::msg::Bool>(
        "/reset", 10, std::bind(&digging::reset, this, std::placeholders::_1));
        cmd_actuator_state_pub = create_publisher<std_msgs::msg::String>("actuator_state", 10);
        
        
    }
    double velocity_actuator = 0.0;
    bool topLimitLeftPressed = false, topLimitRightPressed = false;
  private:
    SparkMax leadscrew_left, leadscrew_right, actuator_left, actuator_right, digging_left, digging_right;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr leadscrew_velocity_subscriber, actuator_velocity_subscriber, digging_velocity_subscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr cmd_state_subscriber;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_drivebase_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr cmd_vel_leadscrew_pub, cmd_vel_actuator_pub, cmd_vel_digging_pub, cmd_vel_dumping_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr cmd_state_pub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cmd_actuator_state_pub;
    rclcpp::TimerBase::SharedPtr timer_diagnostics;
    double actuator_max_travel = 1.5, actuator_min_travel = 0.28; //0.019
    

    enum class LinearActuatorStateLeft {
    Unknown, Raised, Lowered, Traveling, Commanded, Stopped
    };

    enum class LinearActuatorStateRight {
    Unknown, Raised, Lowered, Traveling, Commanded, Stopped
    };

    LinearActuatorStateLeft linear_actuator_state_left;
    LinearActuatorStateRight linear_actuator_state_right;
    
    std::string stateToStringActuatorLeft(LinearActuatorStateLeft state){
        switch (state) {
            case LinearActuatorStateLeft::Raised: return "Raised";
            case LinearActuatorStateLeft::Lowered: return "Lowered";
            case LinearActuatorStateLeft::Traveling: return "Traveling";
            case LinearActuatorStateLeft::Commanded: return "Commanded";
            case LinearActuatorStateLeft::Stopped: return "Stopped";
            default: return "Unknown";
        }
    }
    std::string stateToStringActuatorRight(LinearActuatorStateRight state){
        switch (state) {
            case LinearActuatorStateRight::Raised: return "Raised";
            case LinearActuatorStateRight::Lowered: return "Lowered";
            case LinearActuatorStateRight::Traveling: return "Traveling";
            case LinearActuatorStateRight::Commanded: return "Commanded";
            case LinearActuatorStateRight::Stopped: return "Stopped";
            default: return "Unknown";
        }
    }
    /**
     * @brief Fault IDs based on REV documentation. Use GetFaults() method from Sparkcan
     */
    enum class FaultBits : uint16_t {
        // kBrownout = 0,        // Brownout
        // kSoftLimitClamp = 1,
        // kOtherFault = 2,
        // kDRVFault = 3,
        // kHasReset = 4,
        // kCANRX = 5,
        // kCANTX = 6,
        // KEEPROMCRC = 7,
        // kStall = 8,
        // kSensorFault = 9,
        // kMotorFault = 10,
        // kSoftLimitRev = 11,
        // kSoftLimitFwd = 12,
        kHardLimitFwd = 14,
        kHardLimitRev = 15
    };
    void periodic(){
      checkLimitSwitches();
      checkActuatorPosition();
    }
    void velocity_callback_leadscrew(const std_msgs::msg::Float64::SharedPtr velocity_msg)
    {
        double velocity = velocity_msg->data;
        // RCLCPP_INFO(get_logger(), "leadscrew speed :%lf", velocity);
        leadscrew_left.SetDutyCycle(velocity);
        leadscrew_right.SetDutyCycle(velocity);
    }

    void velocity_callback_actuator(const std_msgs::msg::Float64::SharedPtr velocity_msg){
        velocity_actuator= velocity_msg->data;
        actuator_left.SetDutyCycle(velocity_actuator);
        actuator_right.SetDutyCycle(velocity_actuator);
    }
    void velocity_callback_digging(const std_msgs::msg::Float64::SharedPtr velocity_msg){
      double velocity = velocity_msg->data;
      digging_left.SetDutyCycle(velocity);
      digging_right.SetDutyCycle(-velocity);
    }
    void reset(const std_msgs::msg::Bool::SharedPtr reset_msg){
      if(reset_msg->data == false){
        idleRest();
      }
    }
    void initialization(){
      // bool topLimitLeftPressed = checkFault(leadscrew_left.GetFaults(), FaultBits::kHardLimitFwd), topLimitRightPressed = checkFault(leadscrew_right.GetFaults(), FaultBits::kHardLimitFwd);
      RCLCPP_INFO(get_logger(), "Putting Digging system to rest position");
      digging_left.SetDutyCycle(0.0);
      digging_right.SetDutyCycle(0.0);
      rclcpp::Time start_time = this->get_clock()->now();
        while ((this->get_clock()->now() - start_time).seconds() < 3.0) {
            leadscrew_left.SetDutyCycle(-0.5);
            leadscrew_right.SetDutyCycle(-0.5);
        }
        while((this->get_clock()->now() - start_time).seconds() > 3.0 && (this->get_clock()->now() - start_time).seconds() < 4.5){
            leadscrew_left.SetDutyCycle(0.0);
            leadscrew_right.SetDutyCycle(0.0);
        }
        while (!topLimitLeftPressed && !topLimitRightPressed) {
            leadscrew_left.SetDutyCycle(0.75);
            leadscrew_right.SetDutyCycle(0.75);
            topLimitLeftPressed = checkFault(leadscrew_left.GetFaults(), FaultBits::kHardLimitFwd);
            topLimitRightPressed = checkFault(leadscrew_right.GetFaults(), FaultBits::kHardLimitFwd);
            RCLCPP_INFO(get_logger(), "left :%s right :%s", topLimitLeftPressed ? "active" : "not active", topLimitRightPressed ? "active" : "not active");

        }
        leadscrew_left.SetDutyCycle(0.0);
        leadscrew_right.SetDutyCycle(0.0);
        while(linear_actuator_state_left != LinearActuatorStateLeft::Lowered && linear_actuator_state_right !=LinearActuatorStateRight::Lowered){
          if(actuator_left.GetAnalogPosition() <= actuator_min_travel){
            linear_actuator_state_left = LinearActuatorStateLeft::Lowered;
          }
          if(actuator_right.GetAnalogPosition() <= actuator_min_travel){
            linear_actuator_state_right = LinearActuatorStateRight::Lowered;
          }
          if(linear_actuator_state_left != LinearActuatorStateLeft::Lowered)
              actuator_left.SetDutyCycle(1.0);
          else
              actuator_left.SetDutyCycle(0.0);
          if(linear_actuator_state_right !=LinearActuatorStateRight::Lowered)
              actuator_right.SetDutyCycle(1.0);
          else
              actuator_right.SetDutyCycle(0.0);
          // RCLCPP_INFO(get_logger(), "state left: %s state right: %s" , stateToStringActuatorLeft(linear_actuator_state_left).c_str(), stateToStringActuatorRight(linear_actuator_state_right).c_str());
        }
        
        actuator_left.SetDutyCycle(0.0);
        actuator_right.SetDutyCycle(0.0);

        auto msgReset = std_msgs::msg::Bool();
        msgReset.data = true;
        cmd_state_pub->publish(msgReset);
        RCLCPP_INFO(get_logger(), "Digging system at rest");
    }

    void idleRest(){
      // bool topLimitLeftPressed = checkFault(leadscrew_left.GetFaults(), FaultBits::kHardLimitFwd), topLimitRightPressed = checkFault(leadscrew_right.GetFaults(), FaultBits::kHardLimitFwd);
      RCLCPP_INFO(get_logger(), "Putting Digging system to rest position");
      while (!topLimitLeftPressed && !topLimitRightPressed) {
          leadscrew_left.SetDutyCycle(0.75);
          leadscrew_right.SetDutyCycle(0.75);
          topLimitLeftPressed = checkFault(leadscrew_left.GetFaults(), FaultBits::kHardLimitFwd);
          topLimitRightPressed = checkFault(leadscrew_right.GetFaults(), FaultBits::kHardLimitFwd);
          RCLCPP_INFO(get_logger(), "left :%s right :%s", topLimitLeftPressed ? "active" : "not active", topLimitRightPressed ? "active" : "not active");
      }
      leadscrew_left.SetDutyCycle(0.0);
      leadscrew_right.SetDutyCycle(0.0);

      
      while(linear_actuator_state_left != LinearActuatorStateLeft::Lowered && linear_actuator_state_right !=LinearActuatorStateRight::Lowered){
        if(actuator_left.GetAnalogPosition() <= actuator_min_travel){
          linear_actuator_state_left = LinearActuatorStateLeft::Lowered;
        }
        if(actuator_right.GetAnalogPosition() <= actuator_min_travel){
          linear_actuator_state_right = LinearActuatorStateRight::Lowered;
        }
        if(linear_actuator_state_left != LinearActuatorStateLeft::Lowered)
            actuator_left.SetDutyCycle(1.0);
        else
            actuator_left.SetDutyCycle(0.0);
        if(linear_actuator_state_right !=LinearActuatorStateRight::Lowered)
            actuator_right.SetDutyCycle(1.0);
        else
            actuator_right.SetDutyCycle(0.0);
        // RCLCPP_INFO(get_logger(), "state left: %s state right: %s" , stateToStringActuatorLeft(linear_actuator_state_left).c_str(), stateToStringActuatorRight(linear_actuator_state_right).c_str());
      }
      
      actuator_left.SetDutyCycle(0.0);
      actuator_right.SetDutyCycle(0.0);

      auto msgReset = std_msgs::msg::Bool();
      msgReset.data = true;
      cmd_state_pub->publish(msgReset);
      RCLCPP_INFO(get_logger(), "Digging system at rest");
    }

    bool checkFault(uint16_t faults, FaultBits bit) {
        return (faults & (1 << static_cast<int>(bit)));
    }

    void checkLimitSwitches(){
      uint16_t faults1 = leadscrew_right.GetFaults();
      uint16_t faults2 = leadscrew_left.GetFaults();

      topLimitRightPressed = checkFault(faults1, FaultBits::kHardLimitFwd);
      topLimitLeftPressed = checkFault(faults2, FaultBits::kHardLimitFwd);
      bool bottomLimitRight = checkFault(faults1, FaultBits::kHardLimitRev);
      bool bottomLimitLeft = checkFault(faults2, FaultBits::kHardLimitRev);
      if(topLimitLeftPressed){
          leadscrew_left.SetDutyCycle(0.0);
      }
      if(topLimitRightPressed){
          leadscrew_right.SetDutyCycle(0.0);
      }
      if(bottomLimitLeft){
        leadscrew_left.SetDutyCycle(0.0);
      }
      if(bottomLimitRight){
        leadscrew_right.SetDutyCycle(0.0);
      }
    }

    void checkActuatorPosition(){
      auto msgState = std_msgs::msg::String();
      if(actuator_left.GetAnalogPosition() <= actuator_min_travel){
        linear_actuator_state_left = LinearActuatorStateLeft::Lowered;
      }
      if(actuator_right.GetAnalogPosition() <= actuator_min_travel){
        linear_actuator_state_right = LinearActuatorStateRight::Lowered;
      }
      if(actuator_left.GetAnalogPosition() > actuator_min_travel && actuator_left.GetAnalogPosition() < actuator_max_travel){
        linear_actuator_state_left = LinearActuatorStateLeft::Traveling;
      }
      if(actuator_right.GetAnalogPosition() > actuator_min_travel && actuator_right.GetAnalogPosition() < actuator_max_travel){
        linear_actuator_state_right = LinearActuatorStateRight::Traveling;
      }
      if(actuator_left.GetAnalogPosition() >= actuator_max_travel){
        linear_actuator_state_left = LinearActuatorStateLeft::Raised;
      }
      if(actuator_right.GetAnalogPosition() >= actuator_max_travel){
        linear_actuator_state_right = LinearActuatorStateRight::Raised;
      }

      if(linear_actuator_state_left == LinearActuatorStateLeft::Lowered && linear_actuator_state_right == LinearActuatorStateRight::Lowered)
          velocity_actuator = std::clamp(velocity_actuator, -1.0 , 0.0);
      else if(linear_actuator_state_left == LinearActuatorStateLeft::Raised && linear_actuator_state_right == LinearActuatorStateRight::Raised)
        velocity_actuator = std::clamp(velocity_actuator, 0.0 , 1.0);
      else
        velocity_actuator = std::clamp(velocity_actuator, -1.0 , 1.0);

      auto msg = std_msgs::msg::Float64();
      msg.data = velocity_actuator;
      cmd_vel_actuator_pub->publish(msg);
      if(stateToStringActuatorLeft(linear_actuator_state_left) == stateToStringActuatorRight(linear_actuator_state_right)){
        msgState.data = stateToStringActuatorLeft(linear_actuator_state_left);
        cmd_actuator_state_pub->publish(msgState);
      }
      // RCLCPP_INFO(get_logger(), "state left: %s state right: %s" , stateToStringActuatorLeft(linear_actuator_state_left).c_str(), stateToStringActuatorRight(linear_actuator_state_right).c_str());
      // RCLCPP_INFO(get_logger(), "left: %lf right: %lf", actuator_left.GetAnalogPosition(), actuator_right.GetAnalogPosition());
    }

    void initMotors(){
      linear_actuator_state_left = LinearActuatorStateLeft::Unknown;
      linear_actuator_state_right = LinearActuatorStateRight::Unknown;
      RCLCPP_INFO(get_logger(), "Starting Digging system initialization");
        // Leadscrews
        leadscrew_left.SetIdleMode(IdleMode::kCoast);
        leadscrew_left.SetMotorType(MotorType::kBrushless);
        leadscrew_left.ClearStickyFaults();
        leadscrew_left.SetDutyCycle(0.0);
        leadscrew_left.ResetFaults();
        leadscrew_left.BurnFlash();

        leadscrew_right.SetIdleMode(IdleMode::kCoast);
        leadscrew_right.SetMotorType(MotorType::kBrushless);
        leadscrew_right.ClearStickyFaults();
        leadscrew_right.SetDutyCycle(0.0);
        leadscrew_right.ResetFaults();
        leadscrew_right.BurnFlash();

        actuator_left.SetIdleMode(IdleMode::kCoast);
        actuator_left.SetMotorType(MotorType::kBrushed);
        actuator_left.SetSensorType(SensorType::kEncoder);
        actuator_left.ClearStickyFaults();
        actuator_left.SetDutyCycle(0.0);
        actuator_left.ResetFaults();
        actuator_left.BurnFlash();

        actuator_right.SetIdleMode(IdleMode::kCoast);
        actuator_right.SetMotorType(MotorType::kBrushed);
        actuator_right.SetSensorType(SensorType::kEncoder);
        actuator_right.ClearStickyFaults();
        actuator_right.SetDutyCycle(0.0);
        actuator_right.ResetFaults();
        actuator_right.BurnFlash();

        digging_left.SetIdleMode(IdleMode::kCoast);
        digging_left.SetMotorType(MotorType::kBrushless);
        digging_left.ClearStickyFaults();
        digging_left.SetDutyCycle(0.0);
        digging_left.ResetFaults();
        digging_left.BurnFlash();

        digging_right.SetIdleMode(IdleMode::kCoast);
        digging_right.SetMotorType(MotorType::kBrushless);
        digging_right.ClearStickyFaults();
        digging_right.SetDutyCycle(0.0);
        digging_right.ResetFaults();
        digging_right.BurnFlash();


        leadscrew_left.SetLimitSwitchFwdPolarity(true);  // NC = true
        leadscrew_right.SetLimitSwitchFwdPolarity(true);
        
        // Config reverse limit switches as normally open
        leadscrew_left.SetLimitSwitchRevPolarity(false); // NO = false
        leadscrew_right.SetLimitSwitchRevPolarity(false);
        
        leadscrew_left.SetHardLimitFwdEn(true);
        leadscrew_left.SetHardLimitRevEn(true);

        leadscrew_right.SetHardLimitFwdEn(true);
        leadscrew_right.SetHardLimitRevEn(true);

        RCLCPP_INFO(get_logger(), "Limit switches configured");


        initialization();
        
        RCLCPP_INFO(get_logger(), "Finished Digging system initialization");
    }
  };

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<digging>());
  rclcpp::shutdown();
  return 0;
}
