#include "DiggingLeadscrew.hpp"
#include <std_msgs/msg/string.hpp>


int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DiggingLeadscrew>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

DiggingLeadscrew::DiggingLeadscrew() 
    : Node("digging_leadscrew")
    , m_leadscrew1("can0", LEADSCREW_1_CAN_ID)
    , m_leadscrew2("can0", LEADSCREW_2_CAN_ID)
{
    configureLimitSwitches();
    
    // Set up follower config
    m_leadscrew2.SetFollowerID(LEADSCREW_1_CAN_ID);
    m_leadscrew2.SetFollowerConfig(1);  // Basic follower mode
    
    m_leadscrew1.BurnFlash();
    m_leadscrew2.BurnFlash();

    state_publisher = create_publisher<std_msgs::msg::String>("leadscrew/state", 10);
    timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&DiggingLeadscrew::periodic, this));
    // make these run 1x or periodically??? 
    leadscrew_speed_sub = create_subscription<std_msgs::msg::Float64>("mining/leadscrew_speed", 10,std::bind(&DiggingLeadscrew::handleSpeedCommand, this, std::placeholders::_1));
    belt_speed_sub = create_subscription<std_msgs::msg::Float64>("mining/belt_speed", 10,std::bind(&DiggingLeadscrew::handleSpeedCommand, this, std::placeholders::_1));
    
    RCLCPP_INFO(get_logger(), "Digging Leadscrew initialized");
}

Float64Subscriber leadscrew_speed_sub;
Float64Subscriber belt_speed_sub;

void handleSpeedCommand(const std_msgs::msg::Float64::SharedPtr msg) {
    setLeadscrewSpeed(msg->data); 
}

void DiggingLeadscrew::configureLimitSwitches() {
    // Config forward limit switches as normally closed
    m_leadscrew1.SetLimitSwitchFwdPolarity(true);  // NC = true
    m_leadscrew2.SetLimitSwitchFwdPolarity(true);
    
    // Config reverse limit switches as normally open
    m_leadscrew1.SetLimitSwitchRevPolarity(false); // NO = false
    m_leadscrew2.SetLimitSwitchRevPolarity(false);
    
    m_leadscrew1.SetHardLimitFwdEn(true);
    m_leadscrew1.SetHardLimitRevEn(true);
    
    m_leadscrew2.SetHardLimitFwdEn(true);
    m_leadscrew2.SetHardLimitRevEn(true);

    RCLCPP_INFO(get_logger(), "Limit switches configured");
}

bool DiggingLeadscrew::checkFault(uint16_t faults, FaultBits bit) {
    return (faults & (1 << static_cast<int>(bit))) != 0;
}

void DiggingLeadscrew::periodic() {
    checkLimits();
    publishState();
}

void DiggingLeadscrew::checkLimits() {
    auto position = m_leadscrew1.GetPosition();
    uint16_t faults1 = m_leadscrew1.GetFaults();
    uint16_t faults2 = m_leadscrew2.GetFaults();

    bool topLimit = checkFault(faults1, FaultBits::kHardLimitFwd) || checkFault(faults2, FaultBits::kHardLimitFwd);
    bool bottomLimit = checkFault(faults1, FaultBits::kHardLimitRev) || checkFault(faults2, FaultBits::kHardLimitRev);

    if (topLimit || bottomLimit) {
        RCLCPP_DEBUG(get_logger(), "Limit switch triggered - Top: %s, Bottom: %s", topLimit ? "true" : "false", bottomLimit ? "true" : "false");
    }

    // Update state based on position and limits
    if (leadscrew_state != LeadscrewState::FullExtended && position >= LEADSCREW_MAX_TRAVEL - LEADSCREW_MAX_ERROR) {
        leadscrew_state = LeadscrewState::FullExtended;
        RCLCPP_INFO(get_logger(), "Leadscrew reached full extension");
    }
    else if (leadscrew_state != LeadscrewState::Retracted && (position < LEADSCREW_MAX_ERROR || topLimit)) {
        leadscrew_state = LeadscrewState::Retracted;
        RCLCPP_INFO(get_logger(), "Leadscrew fully retracted");
    }
    else if (leadscrew_state != LeadscrewState::Extended && position > LEADSCREW_MAX_ERROR &&std::abs(position - LEADSCREW_MAX_TRAVEL) >= LEADSCREW_MAX_ERROR) {
        leadscrew_state = LeadscrewState::Extended;
    }
}

void DiggingLeadscrew::publishState() {
    auto msg = std_msgs::msg::String();
    msg.data = stateToString(leadscrew_state);
    state_publisher->publish(msg);
}

std::string DiggingLeadscrew::stateToString(LeadscrewState state) {
    switch (state) {
        case LeadscrewState::Extended: return "Extended";
        case LeadscrewState::Retracted: return "Retracted";
        case LeadscrewState::Traveling: return "Traveling";
        case LeadscrewState::FullExtended: return "FullExtended";
        case LeadscrewState::GivenCommand: return "GivenCommand";
        default: return "Unknown";
    }
}

bool DiggingLeadscrew::isTopLimitPressed() {
    uint16_t faults1 = m_leadscrew1.GetFaults();
    uint16_t faults2 = m_leadscrew2.GetFaults();
    return checkFault(faults1, FaultBits::kHardLimitFwd) || checkFault(faults2, FaultBits::kHardLimitFwd);
}

bool DiggingLeadscrew::isBottomLimitPressed() {
    uint16_t faults1 = m_leadscrew1.GetFaults();
    uint16_t faults2 = m_leadscrew2.GetFaults();
    return checkFault(faults1, FaultBits::kHardLimitRev) || checkFault(faults2, FaultBits::kHardLimitRev);
}

void DiggingLeadscrew::setLeadscrewSpeed(double speed) {
    auto position = m_leadscrew1.GetPosition();
    
    if (position <= LEADSCREW_MAX_ERROR && speed < 0) {
        RCLCPP_WARN(get_logger(), "At bottom limit, cannot move down further");
        return;
    }
    
    if (position >= LEADSCREW_MAX_TRAVEL - LEADSCREW_MAX_ERROR && speed > 0) {
        RCLCPP_WARN(get_logger(), "At top limit, cannot move up further");
        return;
    }

    leadscrew_state = LeadscrewState::Traveling;
    m_leadscrew1.SetDutyCycle(speed);  // Leadscrew2 follows automatically due to follower config
}

LeadscrewState DiggingLeadscrew::getState() const {
    return leadscrew_state;
}