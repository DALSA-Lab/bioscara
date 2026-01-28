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

#ifndef GRIPPER_HARDWARE_HPP_
#define GRIPPER_HARDWARE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <set>
#include <unordered_map>
#include <memory>
#include <limits>

#include "bioscara_gripper_hardware_driver/mGripper.h"
#include "bioscara_gripper_hardware_driver/mMockGripper.h"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace bioscara_hardware_interfaces
{

    /**
     * @brief The bioscara gripper hardware interface class.
     *
     * System interface has been chosen to allow future modifications for grippers that provide feedback.
     * refer to the BioscaraArmHardwareInterface class for a more detailed description of the hardware life-cycle.
     */
    class BioscaraGripperHardwareInterface : public hardware_interface::SystemInterface
    {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(BioscaraGripperHardwareInterface)

        hardware_interface::CallbackReturn on_init(
            const hardware_interface::HardwareComponentInterfaceParams &params) override;

        /**
         * @brief Called on the transistion from the `inactive`, `unconfigured` and `active` to the `finalized` state.
         *
         * When transitioning directly from `active` to `finalized` on_deactivate() is automatically called before [Source Code](https://github.com/ros-controls/ros2_control/blob/d0836b7f12b89acb89bde83d4ba4308513b03204/hardware_interface/src/resource_manager.cpp#L616)
         * If the previous state is either `inactive` or `active` the on_cleanup() method is called first.
         * @param previous_state
         * @return hardware_interface::CallbackReturn
         */
        hardware_interface::CallbackReturn on_shutdown(
            const rclcpp_lifecycle::State &previous_state) override;

        /**
         * @brief Called on the transistion from the `unconfigured` to the `inactive` state.
         *
         * @param previous_state
         * @return hardware_interface::CallbackReturn
         */
        hardware_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State &previous_state) override;

        /**
         * @brief Called on the transistion from the `inactive` to the `unconfigured` state.
         *
         * @param previous_state
         * @return hardware_interface::CallbackReturn
         */
        hardware_interface::CallbackReturn on_cleanup(
            const rclcpp_lifecycle::State &previous_state) override;

        /**
         * @brief Called on the transistion from the `inactive` to the `active` state.
         *
         * Enables PWM generation.
         *
         * @param previous_state
         * @return hardware_interface::CallbackReturn
         */
        hardware_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State &previous_state) override;

        /**
         * @brief Called on the transistion from the `active` to the `inactive` state.
         *
         * Disables PWM generation.
         *
         * @param previous_state
         * @return hardware_interface::CallbackReturn
         */
        hardware_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &previous_state) override;

        /**
         * @brief Reads from the hardware and populates the state interfaces.
         *
         * Invokes the bioscara_hardware_drivers::Gripper::getPosition() method to read the gripper position.
         *
         * @param time
         * @param period
         * @return hardware_interface::return_type
         */
        hardware_interface::return_type read(
            const rclcpp::Time &time,
            const rclcpp::Duration &period) override;

        /**
         * @brief Writes commands to the hardware from the command interfaces.
         *
         * Invokes the bioscara_hardware_drivers::Gripper::setPosition() method to write the command to the gripper.
         *
         * @param time
         * @param period
         * @return hardware_interface::return_type
         */
        hardware_interface::return_type write(
            const rclcpp::Time &time,
            const rclcpp::Duration &period) override;

        /**
         * @brief Called when an error in any state or state transition is thrown.
         *
         * According to the [ros2_control documentation](https://control.ros.org/jazzy/doc/ros2_control/hardware_interface/doc/hardware_components_userdoc.html#handling-of-errors-that-happen-during-read-and-write-calls):
         *
         * > Error handling follows the node lifecycle. If successful CallbackReturn::SUCCESS is returned and hardware is again in `UNCONFIGURED` state, if any ERROR or FAILURE happens the hardware ends in `FINALIZED` state and can not be recovered. The only option is to reload the complete plugin, but there is currently no service for this in the Controller Manager.
         *
         *
         * Since the hardware will immediatly return to the `unconfigured` state ([source](https://github.com/ros-controls/ros2_control/blob/d0836b7f12b89acb89bde83d4ba4308513b03204/hardware_interface/src/hardware_component.cpp#L247))
         * if the error could be handled we manually call the transition functions which would
         * normally be called to this state. Those are:
         * - <b>Previous state</b>: `active`
         *  - Deactivate hardware (on_deactivate()) -> `inactive`
         *  - Clean-Up hardware (on_cleanup()) -> `unconfigured`
         * - <b>Previous state</b>: `inactive`
         *  - Deactivate hardware (on_deactivate()) -> `inactive`
         *      - call the deactivate function anyway regardless if state was active or inactive.
         *  - Clean-Up hardware (on_cleanup()) -> `unconfigured`
         * .
         * @param previous_state
         * @return hardware_interface::CallbackReturn
         */
        hardware_interface::CallbackReturn on_error(
            const rclcpp_lifecycle::State &previous_state) override;

    private:
        /**
         * @brief configuration structure holding the passed paramters from the ros2_control urdf
         *
         * Saving all parameters on initialization in a structure allows for quick access during runtime.
         *
         */
        struct gripper_config_t
        {
            float reduction = 1;
            float offset = 0;
            float min;
            float max;
            float init_pos;
        };

        /**
         * @brief Smart pointer to the local BaseGripper
         *
         * This will be used to either interact with the hardware or mock hardware.
         * A smart pointer is used to guarantee destruction when the pointer is destructed. A unique pointer is used to prevent copying of the object.
         */
        std::unique_ptr<bioscara_hardware_drivers::BaseGripper> _gripper;

        /**
         * @brief configuration struct of the gripper.
         *
         */
        gripper_config_t _gripper_cfg;

        float _last_pos = std::numeric_limits<double>::quiet_NaN();
        float _vel = std::numeric_limits<double>::quiet_NaN();
    };

} // namespace bioscara_hardware_interfaces

#endif // GRIPPER_HARDWARE_HPP_
