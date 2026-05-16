#ifndef INC_VESC_HW_INTERFACE_HPP_
#define INC_VESC_HW_INTERFACE_HPP_

#include "hardware_interface/system_interface.hpp"
#include "vesc.hpp"
#include "rclcpp/rclcpp.hpp"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"


namespace vesc_hw_interface
{
class VescHwInterface : public hardware_interface::SystemInterface
{

public:
    RCLCPP_SHARED_PTR_DEFINITIONS(VescHwInterface)

    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    // std::vector<hardware_interface::CommandInterface::SharedPtr> on_export_command_interfaces() override;

    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
    hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

    std::vector<VescState> read_state;

private:
    vesc driver;
    std::string ifname_;

    std::uint32_t can_id_;
    int number_of_motors;
    std::vector<uint8_t> can_ids_;

    // std::vector<double> hw_commands_velocities_;

    // std::vector<double> hw_states_positions_;
    // std::vector<double> hw_states_velocities_;

    bool delaying;
    std::chrono::_V2::system_clock::time_point begin_time;

    // std::vector<std::uint32_t can_id_;

    std::vector<double> hw_states_positions_;
    std::vector<double> hw_states_velocities_;

    // std::vector<double> hw_commands_positions_;
    std::vector<double> hw_commands_velocities_;
    // std::vector<double> hw_commands_flux_brakes_;

    // std::vector<double> hw_states_modes_;
    // std::vector<double> hw_states_faults_;
    // std::vector<double> hw_states_torques_;
    // std::vector<double> hw_states_voltages_;
    // std::vector<double> hw_states_powers_;
    // std::vector<double> hw_states_board_temperatures_;

    
};
} // moteus_hw_interface
#endif