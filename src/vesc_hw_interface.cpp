#include "vesc_hw_interface/vesc_hw_interface.hpp"

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

// using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using hardware_interface::return_type;

namespace vesc_hw_interface
{

hardware_interface::CallbackReturn VescHwInterface::on_init(const hardware_interface::HardwareInfo & info){
  if(hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS){
      RCLCPP_FATAL(rclcpp::get_logger("VescHW"), "stopped");
      return hardware_interface::CallbackReturn::ERROR;
    }

    hw_states_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_states_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

    // uncommon states
    hw_states_currents_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_states_duties_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_states_temp_fets_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_states_in_currents_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    // hw_states_faults_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    // hw_states_torques_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    // hw_states_voltages_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    // hw_states_powers_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    // hw_states_board_temperatures_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

    // commands
    // hw_commands_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_commands_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    // hw_commands_flux_brakes_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

    // hw_motor_temperature_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    // hw_voltage_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

    // ifname_ = info_.hardware_parameters["ifname"];

    // for (const hardware_interface::ComponentInfo &joint : info_.joints)
    // {
    //   can_id_.emplace_back(std::stoi(joint.parameters.at("can_id")));
    // }

    RCLCPP_INFO(rclcpp::get_logger("VescHW"), "Configuring...");
    // time_ = std::chrono::system_clock::now();

    ifname_ = info_.hardware_parameters["ifname"];
    number_of_motors = stoi(info_.hardware_parameters["number_of_motors"]);
    can_ids_.resize(number_of_motors);
    for(int i = 0; i<number_of_motors;i++){
      std::string id_name = "can_id"+std::to_string(i);
      RCLCPP_INFO(rclcpp::get_logger("VescHW"), "%s id is %d",id_name.c_str(),stoi(info_.hardware_parameters[id_name]));
      can_ids_[i] = stoi(info_.hardware_parameters[id_name]);
    }
    // RCLCPP_INFO(rclcpp::get_logger("MoteusHW"), "IDs: %s",info_.hardware_parameters["can_ids"]);
    // can_ids_ = stoi(info_.hardware_parameters["can_ids"]);

    return CallbackReturn::SUCCESS;
  }

hardware_interface::CallbackReturn VescHwInterface::on_configure(const rclcpp_lifecycle::State &/*previous_state*/){
  // for(int i = 0; i<can_ids_.size();i++){
  //   RCLCPP_INFO(rclcpp::get_logger("MoteusHW"), "%d id is %d",i,can_ids_[i]);
  // }
  RCLCPP_INFO(rclcpp::get_logger("VescHW"), "number: %d",can_ids_.size());
  driver.setup(can_ids_,ifname_);

  // reset values always when configuring hardware
  // for (const auto & [name, descr] : joint_state_interfaces_)
  // {
  //   set_state(name, 0.0);
  // }
  // for (const auto & [name, descr] : joint_command_interfaces_)
  // {
  //   set_command(name, 0.0);
  // }
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn VescHwInterface::on_activate(const rclcpp_lifecycle::State &/*previous_state*/){
  return CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn VescHwInterface::on_deactivate(const rclcpp_lifecycle::State &/*previous_state*/){
  driver.deactivate();
  return CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::StateInterface> VescHwInterface::export_state_interfaces(){
  // RCLCPP_INFO(rclcpp::get_logger("MoteusHW"), "Configuring state ifaces");

  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++){
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocities_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_positions_[i]));

    // uncommon states
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, "current", &hw_states_currents_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, "duty", &hw_states_duties_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, "temp_fet", &hw_states_temp_fets_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, "current_in", &hw_states_in_currents_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> VescHwInterface::export_command_interfaces(){
  // RCLCPP_INFO(rclcpp::get_logger("MoteusHW"), "Configuring command ifaces");

  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++){
    command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_velocities_[i]));
    // command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_positions_[i]));
    // command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, "flux_brake", &hw_commands_flux_brakes_[i]));
  }
  return command_interfaces;
}

hardware_interface::return_type VescHwInterface::read(const rclcpp::Time &, const rclcpp::Duration &){
  read_state = driver.get_state();
  for (auto i = 0u; i < info_.joints.size(); i++){
    hw_states_velocities_[i] = (double)read_state[i].velocity;
    hw_states_positions_[i] = (double)read_state[i].position;

    // uncommon states
    hw_states_currents_[i] = (double)read_state[i].current;
    hw_states_duties_[i] = (double)read_state[i].duty;
    hw_states_temp_fets_[i] = (double)read_state[i].temp_fet;
    hw_states_in_currents_[i] = (double)read_state[i].in_current;
    // hw_states_voltages_[i] = (double)read_state[i].voltage;
  }

  return return_type::OK;
}

hardware_interface::return_type VescHwInterface::write(const rclcpp::Time &, const rclcpp::Duration &){
  // for (std::size_t i = 0; i < info_.joints.size(); i++){
    // driver.write_velocity(hw_commands_velocities_[0]);
  // }
  bool all_zero = true;
  for (auto i = 0u; i < info_.joints.size(); i++){
    if(hw_commands_velocities_[i] != 0.0){
      all_zero = false;
      break;
    }
  }
  
  if(all_zero){
    driver.write_stop();
  }else{
    driver.write_velocity(hw_commands_velocities_);
  }

  // double target_velocity = hw_commands_velocities_[0];
  // if(target_velocity == 0.0){
  //   if(delaying){
  //     begin_time = std::chrono::high_resolution_clock::now();
  //     delaying = false;
  //   }else{
  //     if(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - begin_time).count() >= 1000){
  //       double flux_brake_state = hw_commands_flux_brakes_[0];
  //       if(flux_brake_state > 0.0){
  //         driver.write_brake();
  //       }else if(flux_brake_state == 0.0){
  //         driver.write_stop();
  //       }
  //     }else{
  //       driver.write_stop();
  //     }
  //   }
    // double flux_brake_state = get_command(descr.get_prefix_name()+"/flux_brake");
    // // RCLCPP_INFO(rclcpp::get_logger("MoteusHW"),"Flux brake state: %f",flux_brake_state);
    // if(flux_brake_state > 0.0){
    //   driver.write_brake();
    // }else if(flux_brake_state == 0.0){
    //   driver.write_stop();
  // }else{
  //   if(!delaying){
  //     begin_time = std::chrono::high_resolution_clock::now();
  //     delaying = true;
  //   }else{
  //     if(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - begin_time).count() >= 1000){
  //       driver.write_velocity(target_velocity);
  //     }else{
  //       driver.write_stop();
  //     }
  //   }
  // }
  return return_type::OK;
}
} // namespace moteus_hw_interface


#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  vesc_hw_interface::VescHwInterface,
  hardware_interface::SystemInterface
)