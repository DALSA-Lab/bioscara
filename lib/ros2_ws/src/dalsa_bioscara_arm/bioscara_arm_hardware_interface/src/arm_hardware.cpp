// Copyright 2023 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "bioscara_arm_hardware_interface/arm_hardware.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace bioscara_hardware_interfaces
{
  hardware_interface::CallbackReturn BioscaraArmHardwareInterface::on_init(
      const hardware_interface::HardwareComponentInterfaceParams &params)
  {
    if (
        hardware_interface::SystemInterface::on_init(params) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    std::string use_mock_hardware = info_.hardware_parameters["use_mock_hardware"];

    _ordered_joint_state_interfaces_ptr.clear();

    /**
     * Loop over all joints decribed in the hardware description file, check if they have the position and velocity command
     * and state interface defined and finally add them to the internal _joints list
     *
     */
    for (const hardware_interface::ComponentInfo &joint : info_.joints)
    {
      // expect exactly two command interface
      if (joint.command_interfaces.size() != 3)
      {
        RCLCPP_FATAL(
            get_logger(), "Joint '%s' has %zu command interfaces found. 3 expected.",
            joint.name.c_str(), joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      // expect the first command interface to be position
      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
            get_logger(), "Joint '%s' has '%s' command interface. '%s' expected.",
            joint.name.c_str(),
            joint.command_interfaces[0].name.c_str(),
            hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }

      // expect the second command interface to be velocity
      if (joint.command_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
            get_logger(), "Joint '%s' has '%s' command interface. '%s' expected.",
            joint.name.c_str(),
            joint.command_interfaces[0].name.c_str(),
            hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }

      // expect the third command interface to be home
      if (joint.command_interfaces[2].name != bioscara_hardware_interfaces::HW_IF_HOME)
      {
        RCLCPP_FATAL(
            get_logger(), "Joint '%s' has '%s' command interface. '%s' expected.",
            joint.name.c_str(),
            joint.command_interfaces[0].name.c_str(),
            bioscara_hardware_interfaces::HW_IF_HOME);
        return hardware_interface::CallbackReturn::ERROR;
      }

      // expect exactly three state interfaces
      if (joint.state_interfaces.size() != 3)
      {
        RCLCPP_FATAL(
            get_logger(), "Joint '%s' has %zu state interface. 3 expected.", joint.name.c_str(),
            joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      // expect state interface to be position or velocity
      if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION &&
          joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY &&
          joint.state_interfaces[2].name != bioscara_hardware_interfaces::HW_IF_HOME)
      {
        RCLCPP_FATAL(
            get_logger(), "Joint '%s' have %s state interfaces found. '%s', '%s' and '%s' expected (in this order).",
            joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
            hardware_interface::HW_IF_POSITION,
            hardware_interface::HW_IF_VELOCITY,
            bioscara_hardware_interfaces::HW_IF_HOME);
        return hardware_interface::CallbackReturn::ERROR;
      }

      /* We now know that the joint has a position, velocity and homing state interface.
      Now populate the vector to point to the corresponding interfaces in the correct order.
      */
      for (auto &interface_name : {joint.name + "/" + hardware_interface::HW_IF_POSITION,
                                   joint.name + "/" + hardware_interface::HW_IF_VELOCITY,
                                   joint.name + "/" + bioscara_hardware_interfaces::HW_IF_HOME})
      {
        _ordered_joint_state_interfaces_ptr.push_back({interface_name,&joint_state_interfaces_.at(interface_name)});
      }

      // add joint one by one reading parameters from urdf
      joint_config_t cfg;
      try
      {
        cfg.i2c_address = std::stoi(joint.parameters.at("i2c_address"), nullptr, 16);
        cfg.reduction = std::stof(joint.parameters.at("reduction"));
        cfg.min = std::stof(joint.parameters.at("min"));
        cfg.max = std::stof(joint.parameters.at("max"));
        cfg.stall_threshold = std::stoi(joint.parameters.at("stall_threshold"));
        cfg.hold_current = std::stoi(joint.parameters.at("hold_current"));
        cfg.drive_current = std::stoi(joint.parameters.at("drive_current"));
        cfg.max_acceleration = std::stof(joint.parameters.at("max_acceleration"));
        cfg.max_velocity = std::stof(joint.parameters.at("max_velocity"));

        /**
         * @todo threshold and current are uint8_t, if a number larger outside 0 < n < 255 is passed as a parameters it will overflow.
         */
        try
        {
          cfg.homing.speed = std::stof(joint.command_interfaces[2].parameters.at("speed"));
          cfg.homing.threshold = std::stoi(joint.command_interfaces[2].parameters.at("threshold"));
          cfg.homing.current = std::stoi(joint.command_interfaces[2].parameters.at("current"));
          cfg.homing.acceleration = std::stof(joint.command_interfaces[2].parameters.at("acceleration"));
        }
        catch (...)
        {
          RCLCPP_FATAL(
              get_logger(), "Joint '%s' is missing one of the following parameters in 'home' command_interface: speed, threshold, current, acceleration",
              joint.name.c_str());
          return hardware_interface::CallbackReturn::ERROR;
        }
      }
      catch (...)
      {
        RCLCPP_FATAL(
            get_logger(), "Joint '%s' is missing one of the following parameters: i2c_address, reduction, min, max, stall_threshold, hold_current, drive_current, max_acceleration, max_velocity, homing",
            joint.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }

      _joint_cfg.insert({joint.name, cfg});

      if (use_mock_hardware == "true" || use_mock_hardware == "True")
      {
        /* construct the object directly in the map */
        _joints.emplace(joint.name, std::make_unique<bioscara_hardware_drivers::MockJoint>(joint.name));
      }
      else
      {
        _joints.emplace(joint.name, std::make_unique<bioscara_hardware_drivers::Joint>(joint.name,
                                                                                       cfg.i2c_address,
                                                                                       cfg.reduction,
                                                                                       cfg.min,
                                                                                       cfg.max));
      }
    }
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn BioscaraArmHardwareInterface::on_shutdown(
      const rclcpp_lifecycle::State &previous_state)
  {
    RCLCPP_INFO(get_logger(), "Shutting down ...please wait...");

    hardware_interface::CallbackReturn cr = hardware_interface::CallbackReturn::SUCCESS;
    switch (previous_state.id())
    {
      /* Joints already deinitialized */
    case lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED:
      break;

      /* Deinitialize joints first, TODO: necessary to call deactivate() as well? */
    case lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE:
    case lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE:
      cr = on_cleanup(previous_state);
      break;
    }
    if (cr != hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }
    _joints.clear();
    RCLCPP_INFO(get_logger(), "Shut down");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn BioscaraArmHardwareInterface::on_configure(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(get_logger(), "Configuring ...please wait...");

    // Init each joint and test connection to each joint by pinging
    for (auto &[name, joint] : _joints)
    {
      bioscara_hardware_drivers::err_type_t rc = joint->init();
      if (rc != bioscara_hardware_drivers::err_type_t::OK)
      {
        std::string reason = bioscara_hardware_drivers::error_to_string(rc);
        RCLCPP_ERROR(
            get_logger(),
            "Failed to connect to joint '%s'. Reason: %s", name.c_str(), reason.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    // reset values always when configuring hardware
    for (const auto &[name, descr] : joint_state_interfaces_)
    {
      set_state(name, 0.0);
    }
    for (const auto &[name, descr] : joint_command_interfaces_)
    {
      set_command(name, 0.0);
    }
    for (const auto &[name, descr] : gpio_state_interfaces_)
    {
      set_state(name, 0.0);
    }
    for (const auto &[name, descr] : gpio_command_interfaces_)
    {
      set_command(name, 0.0);
    }
    RCLCPP_INFO(get_logger(), "Successfully configured!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn BioscaraArmHardwareInterface::on_cleanup(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(get_logger(), "Cleaning up ...please wait...");

    /**
     * Disconnect from the joints and throw error if it fails
     */
    for (auto &[name, joint] : _joints)
    {
      bioscara_hardware_drivers::err_type_t rc = joint->deinit();
      if (rc != bioscara_hardware_drivers::err_type_t::OK)
      {
        std::string reason = bioscara_hardware_drivers::error_to_string(rc);
        RCLCPP_ERROR(
            get_logger(),
            "Failed to disconnect from joint '%s'. Reason: %s", name.c_str(), reason.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
    RCLCPP_INFO(get_logger(), "Successfully cleaned up!");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn BioscaraArmHardwareInterface::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(get_logger(), "Activating ...please wait...");

    for (auto &[name, joint] : _joints)
    {
      /* Clear the active command modes. Controllers can only be activated after the hardware is activated. */
      _joint_command_modes[name] = {};

      if (activate_joint(name) != bioscara_hardware_drivers::err_type_t::OK)
      {
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    /**
     * Below a workaround to force a read cycle of all joints to get inital values for the state interfaces.
     * These will be copied to the command interface to prevent movement at startup.
     */
    rclcpp::Time t(0);
    rclcpp::Duration d(0, 0);
    if (read(t, d) != hardware_interface::return_type::OK)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    // command and state should be equal when starting
    for (const auto &[name, descr] : joint_command_interfaces_)
    {
      /* Check if position or velocity. Set position to current position and velocity to 0.0 */
      if (descr.interface_info.name == hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_INFO(get_logger(), "Set %s, to %f", name.c_str(), get_state(name));
        set_command(name, get_state(name));
      }
      else if (descr.interface_info.name == hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_INFO(get_logger(), "Set %s, to 0.0", name.c_str());
        set_command(name, 0.0);
      }
      else if (descr.interface_info.name == bioscara_hardware_interfaces::HW_IF_HOME)
      {
        RCLCPP_INFO(get_logger(), "Set %s, to 0.0", name.c_str());
        set_command(name, 0.0);
      }
    }

    RCLCPP_INFO(get_logger(), "Successfully activated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn BioscaraArmHardwareInterface::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(get_logger(), "Deactivating ...please wait...");

    /**
     * disable the joints and throw error if it fails
     *
     */
    for (auto &[name, joint] : _joints)
    {
      if (deactivate_joint(name) != bioscara_hardware_drivers::err_type_t::OK)
      {
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
    RCLCPP_INFO(get_logger(), "Successfully deactivated!");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type BioscaraArmHardwareInterface::read(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    // lock access to _joints and _joint_command_modes
    const std::lock_guard<std::mutex> lock(mtx);

    for (const auto &[name, descr_ptr] : _ordered_joint_state_interfaces_ptr)
    {
      float v;
      bioscara_hardware_drivers::err_type_t rc = static_cast<bioscara_hardware_drivers::err_type_t>(1);
      if (descr_ptr->interface_info.name == hardware_interface::HW_IF_POSITION)
      {
        rc = _joints.at(descr_ptr->prefix_name)->getPosition(v);
      }
      else if (descr_ptr->interface_info.name == hardware_interface::HW_IF_VELOCITY)
      {
        rc = _joints.at(descr_ptr->prefix_name)->getVelocity(v);
      }
      else if (descr_ptr->interface_info.name == bioscara_hardware_interfaces::HW_IF_HOME)
      {
        /* Reset the return code. All following functions do not return an error code */
        rc = bioscara_hardware_drivers::err_type_t::OK;

        /* As we are reading the joint interfaces in order we are guaranteed to have received tha latest flags.
         Hence we can simply retrieve the HOMED flag by calling isHomed().
        This does not generate additional communication trafic.  */
        v = _joints.at(descr_ptr->prefix_name)->isHomed() * 1.0;

        /* If the homing has been activated (through the command interface) the device signals BUSY
        as long as it is still homing. If the BUSY flag is reset while the current command is still HOME
        we can assume the homing has finished. Then stop the homing. */
        if (_joints.at(descr_ptr->prefix_name)->getCurrentBCmd() == bioscara_hardware_drivers::Joint::HOME &&
            !_joints.at(descr_ptr->prefix_name)->isBusy())
        {
          stop_homing(descr_ptr->prefix_name);
          /* reset the command to not immediatly start a new homing.
          Only possible with a controller which updates the command only once (SingleTriggerController) */
          set_command(name, 0.0);
        }
      }

      if (rc != bioscara_hardware_drivers::err_type_t::OK)
      {
        std::string reason = bioscara_hardware_drivers::error_to_string(rc);
        RCLCPP_ERROR(
            get_logger(),
            "Failed to read %s of joint '%s'. Reason: %s", descr_ptr->interface_info.name.c_str(), name.c_str(), reason.c_str());
        return hardware_interface::return_type::DEACTIVATE;
      }
      set_state(name, (double)v);
    }

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type BioscaraArmHardwareInterface::write(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {

    // lock access to _joints and _joint_command_modes
    const std::lock_guard<std::mutex> lock(mtx);

    /* Loop over all active command interfaces for each joint and write the command to the hardware.
    Previously the command was written to every command interface specified in the ros2_control urdf. This causes conflicts. */
    for (const auto &[name, interfaces] : _joint_command_modes)
    {
      for (std::string interface : interfaces)
      {
        std::string CIF_name = name + "/" + interface;
        bioscara_hardware_drivers::err_type_t rc = static_cast<bioscara_hardware_drivers::err_type_t>(1);
        if (interface == hardware_interface::HW_IF_POSITION)
        {
          rc = _joints.at(name)->setPosition((float)get_command(CIF_name));
        }
        else if (interface == hardware_interface::HW_IF_VELOCITY)
        {
          rc = _joints.at(name)->setVelocity((float)get_command(CIF_name));
        }
        else if (interface == bioscara_hardware_interfaces::HW_IF_HOME)
        {
          rc = bioscara_hardware_drivers::err_type_t::OK;

          float velocity = get_command(CIF_name);
          bioscara_hardware_drivers::Joint::stp_reg_t current_cmd = _joints.at(name)->getCurrentBCmd();

          /* If the joint is currently executing a homing call and the velocity is set to 0.0,
          stop the homing. This indicates that the homing should be aborted.*/
          if (velocity == 0.0)
          {
            if (current_cmd == bioscara_hardware_drivers::Joint::HOME)
            {
              rc = stop_homing(name);

              /* In this case, if the homing is manually stopped,
              we expect stop_homing() to return -2 (not homed). Hence this is not an error. */
              if (rc == bioscara_hardware_drivers::err_type_t::NOT_HOMED)
              {
                rc = bioscara_hardware_drivers::err_type_t::OK;
              }
            }
          }

          /* If the command is != 0.0 and the joint is currently not executing a blocking command,
          most likely the homing itself from a previous call, start the homing sequence. */
          else
          {
            if (current_cmd == bioscara_hardware_drivers::Joint::NONE)
            {
              rc = start_homing(name, velocity);
            }
            else if (current_cmd != bioscara_hardware_drivers::Joint::HOME)
            {
              rc = bioscara_hardware_drivers::err_type_t::INCORRECT_STATE;
            }
          }
        }
        // use != 0 here since 1 for no compatible interface type
        if (rc != bioscara_hardware_drivers::err_type_t::OK)
        {
          std::string reason = bioscara_hardware_drivers::error_to_string(rc);
          RCLCPP_ERROR(
              get_logger(),
              "Failed to set %s of joint '%s'. Reason: %s", CIF_name.c_str(), name.c_str(), reason.c_str());
          return hardware_interface::return_type::DEACTIVATE;
        }
      }
    }

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type BioscaraArmHardwareInterface::prepare_command_mode_switch(
      const std::vector<std::string> &start_interfaces,
      const std::vector<std::string> &stop_interfaces)
  {

    /* Dont need to protect _joint_command_modes here since it will only be modified in perform_command_mode_switch,
     which will only be called after prepare_command_mode_switch. */
    _new_joint_command_modes = _joint_command_modes;

    /* First remove all stopped interfaces from the active set */
    for (std::string interface : stop_interfaces)
    {
      std::string full_interface = interface;
      std::string joint;
      split_interface_string_to_joint_and_name(interface, joint, interface);

      /* If the interface that is to be stopped is the homing interface, check that no current homing command is active */
      if (interface == bioscara_hardware_interfaces::HW_IF_HOME &&
          _joints.at(joint)->getCurrentBCmd() == bioscara_hardware_drivers::Joint::HOME)
      {
        RCLCPP_ERROR(
            get_logger(),
            "The controller tried to deactivate '%s' of '%s' but homing is still ongoing.", interface.c_str(), joint.c_str());
        return hardware_interface::return_type::ERROR;
      }

      /* If the interface that is to be stopped is the velocity interface, check that the velocity is 0.0 */
      if (interface == hardware_interface::HW_IF_VELOCITY &&
          get_command(full_interface) != 0.0)
      {
        RCLCPP_WARN(
            get_logger(),
            "The controller tried to deactivate '%s' of '%s' but the velocity is not 0.0", interface.c_str(), joint.c_str());
      }

      if (_new_joint_command_modes.at(joint).erase(interface) == 0)
      {
        RCLCPP_WARN(
            get_logger(),
            "The controller tried to stop the interface '%s' of '%s' but it has not been started.", interface.c_str(), joint.c_str());
      }
    }

    /* Then add all new interfaces to the active set */
    for (std::string interface : start_interfaces)
    {
      std::string joint;
      split_interface_string_to_joint_and_name(interface, joint, interface);

      std::pair rc = _new_joint_command_modes.at(joint).insert(interface);
      if (rc.second == 0)
      {
        RCLCPP_ERROR(
            get_logger(),
            "The controller tried to start the interface '%s' of '%s' but it has already been started by another controller.", interface.c_str(), joint.c_str());
        return hardware_interface::return_type::ERROR;
      }
    }

    /* Validation. Currently the validation is very simple, for every joint only one active interface must exist.*/
    for (auto &[name, interfaces] : _new_joint_command_modes)
    {
      size_t size = interfaces.size();
      if (size > 1)
      {
        std::string active_if = "";
        for (auto interface : interfaces)
        {
          active_if += (interface + "\n");
        }
        RCLCPP_ERROR(
            get_logger(),
            "The controller tries to start multiple command interfaces for '%s'. The following interfaces are active or are trying to be active:\n%s.",
            name.c_str(), active_if.c_str());
        return hardware_interface::return_type::ERROR;
      }

      for (std::string interface : interfaces)
      {
        /* Reject mode switch if joint in question is not homed */
        if (interface == hardware_interface::HW_IF_VELOCITY ||
            interface == hardware_interface::HW_IF_POSITION)
        {
          if (!_joints.at(name)->isHomed())
          {
            RCLCPP_ERROR(
                get_logger(),
                "The controller tried to start the '%s' command interfaces for '%s', which is not homed yet.",
                interface.c_str(), name.c_str());
            return hardware_interface::return_type::ERROR;
          }
        }
        else if (interface == bioscara_hardware_interfaces::HW_IF_HOME)
        {
        }
      }
    }

    /* Print new active interfaces for debugging. */
    std::string active_if = "";
    for (const auto &[name, interfaces] : _new_joint_command_modes)
    {
      active_if += (name + ":\n[\n");
      for (std::string interface : interfaces)
      {
        active_if += ("\t" + interface + "\n");
      }
      active_if += ("]\n");
    }

    RCLCPP_INFO(
        get_logger(),
        "New active command modes:\n%s", active_if.c_str());

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type BioscaraArmHardwareInterface::perform_command_mode_switch(
      const std::vector<std::string> &start_interfaces,
      const std::vector<std::string> &stop_interfaces)
  {
    // lock the mutex, automatically released when leaving scope
    const std::lock_guard<std::mutex> lock(mtx);

    /* If the command mode switch was successfull save the new active interfaces from the cache. */
    _joint_command_modes = _new_joint_command_modes;

    /* We can assume the start and stop interfaces are valid.
    Iterate over each interface and test for its type. Perform action depending on its type for its joint. */
    for (auto interface : start_interfaces)
    {
      std::string joint_name, interface_name;
      split_interface_string_to_joint_and_name(interface, joint_name, interface_name);
      if (interface_name == hardware_interface::HW_IF_POSITION)
      {
      }
      else if (interface_name == hardware_interface::HW_IF_VELOCITY)
      {
      }
      else if (interface_name == bioscara_hardware_interfaces::HW_IF_HOME)
      {
        /* Reset homing command on activation to avoid triggering a homing if some command is left in the command interface */
        set_command(interface, 0.0);

        /* Deactivate joint */
        deactivate_joint(joint_name);
      }
    }
    for (auto interface : stop_interfaces)
    {
      std::string joint_name;
      split_interface_string_to_joint_and_name(interface, joint_name, interface);
      if (interface == hardware_interface::HW_IF_POSITION)
      {
      }
      else if (interface == hardware_interface::HW_IF_VELOCITY)
      {
      }
      else if (interface == bioscara_hardware_interfaces::HW_IF_HOME)
      {
        /* Activate joint again */
        activate_joint(joint_name);
      }
    }
    return hardware_interface::return_type::OK;
  }

  hardware_interface::CallbackReturn BioscaraArmHardwareInterface::on_error(
      const rclcpp_lifecycle::State &previous_state)
  {
    /*
     * Call the deactivation method. If the robot successfully deactivates the hardware remains in the unconfigured state,
     * and is able to be activated again. Otherwise the hardware goes to the finalized state and can not be recovered.
     */
    RCLCPP_INFO(get_logger(), "Previous State: %s", previous_state.label().c_str());
    // states: "active", "finalized",...

    /*
     * call the deactivate function anyway regardless if state was active or inactive. For example if the on_activate function fails
     * the joint might still be enabled, to disable them invoke on_deactivate().
     */
    if (previous_state.label() == "active" || previous_state.label() == "inactive")
    {
      hardware_interface::CallbackReturn cr = on_deactivate(previous_state);
      if (cr != hardware_interface::CallbackReturn::SUCCESS)
      {
        return cr;
      }

      /* since the hardware goes to "unconfigured state if an error is caught and the on_error function returns SUCCESS
      we also have to manually call the on_cleanup function */
      cr = on_cleanup(previous_state);
      if (cr != hardware_interface::CallbackReturn::SUCCESS)
      {
        return cr;
      }
      return hardware_interface::CallbackReturn::SUCCESS;
    }

    return hardware_interface::CallbackReturn::ERROR;
  }

  bioscara_hardware_drivers::err_type_t BioscaraArmHardwareInterface::start_homing(const std::string name, float velocity)
  {
    joint_config_t cfg = _joint_cfg[name];

    float speed = velocity > 0.0 ? cfg.homing.speed : -cfg.homing.speed;

    /* Activate the joint, set homing acceleration and start homing. */
    RETURN_ON_ERROR(activate_joint(name));
    RETURN_ON_ERROR(_joints.at(name)->setMaxAcceleration(cfg.homing.acceleration));
    RETURN_ON_ERROR(_joints.at(name)->startHoming(speed, cfg.homing.threshold, cfg.homing.current));

    RCLCPP_INFO(get_logger(), "Started homing joint '%s'", name.c_str());
    return bioscara_hardware_drivers::err_type_t::OK;
  }

  bioscara_hardware_drivers::err_type_t BioscaraArmHardwareInterface::stop_homing(const std::string name)
  {
    joint_config_t cfg = _joint_cfg[name];

    /* Stop the homing. Reset acceleration and velocity and perform the postHoming cleanup, then deactivate the joint. */
    RETURN_ON_ERROR(_joints.at(name)->setMaxAcceleration(cfg.max_acceleration));
    RETURN_ON_ERROR(_joints.at(name)->setMaxVelocity(cfg.max_velocity));
    RETURN_ON_ERROR(_joints.at(name)->stop());
    _joints.at(name)->postHoming();
    RETURN_ON_ERROR(deactivate_joint(name));

    RCLCPP_INFO(get_logger(), "Finished homing joint '%s'", name.c_str());

    return bioscara_hardware_drivers::err_type_t::OK;
  }

  void BioscaraArmHardwareInterface::split_interface_string_to_joint_and_name(std::string interface, std::string &joint_name, std::string &interface_name)
  {
    /* split 'joint/interface to 'joint' and 'interface' */
    std::string delimiter = "/";
    size_t pos = interface.find(delimiter);
    joint_name = interface.substr(0, pos);
    interface.erase(0, pos + delimiter.length());
    interface_name = interface;
  }

  bioscara_hardware_drivers::err_type_t BioscaraArmHardwareInterface::activate_joint(const std::string name)
  {
    joint_config_t cfg = _joint_cfg[name];

    // enable motor
    bioscara_hardware_drivers::err_type_t rc = _joints.at(name)->enable(cfg.drive_current, cfg.hold_current);
    if (rc != bioscara_hardware_drivers::err_type_t::OK)
    {
      std::string reason = bioscara_hardware_drivers::error_to_string(rc);
      RCLCPP_ERROR(
          get_logger(),
          "Failed to enable joint '%s'. Reason: %s", name.c_str(), reason.c_str());
      return bioscara_hardware_drivers::err_type_t::ERROR;
    }

    if (cfg.stall_threshold > 0)
    {
      // enable stall detection
      rc = _joints.at(name)->enableStallguard(cfg.stall_threshold);
      if (rc != bioscara_hardware_drivers::err_type_t::OK)
      {
        std::string reason = bioscara_hardware_drivers::error_to_string(rc);
        RCLCPP_ERROR(
            get_logger(),
            "Failed to enable stall protection of joint '%s'. Reason: %s", name.c_str(), reason.c_str());
        return bioscara_hardware_drivers::err_type_t::ERROR;
      }
    }

    // set max acceleration
    rc = _joints.at(name)->setMaxAcceleration(cfg.max_acceleration);
    if (rc != bioscara_hardware_drivers::err_type_t::OK)
    {
      std::string reason = bioscara_hardware_drivers::error_to_string(rc);
      RCLCPP_ERROR(
          get_logger(),
          "Failed to set maximum acceleration of joint '%s'. Reason: %s", name.c_str(), reason.c_str());
      return bioscara_hardware_drivers::err_type_t::ERROR;
    }

    // set max velocity
    rc = _joints.at(name)->setMaxVelocity(cfg.max_velocity);
    if (rc != bioscara_hardware_drivers::err_type_t::OK)
    {
      std::string reason = bioscara_hardware_drivers::error_to_string(rc);
      RCLCPP_ERROR(
          get_logger(),
          "Failed to set maximum velocity of joint '%s'. Reason: %s", name.c_str(), reason.c_str());
      return bioscara_hardware_drivers::err_type_t::ERROR;
    }

    return bioscara_hardware_drivers::err_type_t::OK;
  }

  bioscara_hardware_drivers::err_type_t BioscaraArmHardwareInterface::deactivate_joint(const std::string name)
  {
    bioscara_hardware_drivers::err_type_t rc = _joints.at(name)->disable();
    if (rc != bioscara_hardware_drivers::err_type_t::OK)
    {
      std::string reason = bioscara_hardware_drivers::error_to_string(rc);
      RCLCPP_ERROR(
          get_logger(),
          "Failed to disable joint '%s'. Reason: %s", name.c_str(), reason.c_str());
      return bioscara_hardware_drivers::err_type_t::ERROR;
    }
    return bioscara_hardware_drivers::err_type_t::OK;
  }

} // namespace bioscara_hardware_interfaces

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    bioscara_hardware_interfaces::BioscaraArmHardwareInterface, hardware_interface::SystemInterface)
