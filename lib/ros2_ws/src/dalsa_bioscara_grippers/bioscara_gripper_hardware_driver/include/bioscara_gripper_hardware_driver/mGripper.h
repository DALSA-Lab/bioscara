/**
 * @file mGripper.h
 * @author sbstorz
 * @brief File containing the Gripper class
 * @version 0.1
 * @date 2025-05-27
 *
 * @copyright Copyright (c) 2025

 * Include this file for API functions to interact with the hardware gripper.
 *
 */
#ifndef MGRIPPER_H
#define MGRIPPER_H
#include "bioscara_gripper_hardware_driver/mBaseGripper.h"
#include "bioscara_gripper_hardware_driver/uPWM.h"
#include "bioscara_arm_hardware_driver/uErr.h"

namespace bioscara_hardware_drivers
{
    /**
     * @brief Child class implementing control of the hardware gripper.
     *
     * Use this class if you wish to control the actual hardware by generating a PWM voltage on GPIO18 of the Raspberry Pi.
     *
     */
    class Gripper : public BaseGripper
    {
    public:
        /**
         * @brief Construct a new Gripper object
         *
         * see the BaseGripper constructor for a description of the parameters.
         */
        Gripper(float reduction, float offset, float min, float max, float backup_init_pos);

        /**
         * @brief Activates the the hardware.
         *
         * Invokes BaseGripper::enable() and starts the PWM generation but does not set a position.
         * Must be called before a position is set.
         * The PWM pin is GPIO18 on the Raspberry Pi 4. PWM chip is 0, channel 0.
         *
         * @return err_type_t::OK on success, err_type_t::COMM_ERROR if PWM can not be enabled
         */
        err_type_t enable(void) override;

        /**
         * @brief Deactivates the hardware.
         *
         * Invokes BaseGripper::disable()
         * and stops the servo by disableing the PWM generation.
         *
         * @return err_type_t::OK
         */
        err_type_t disable(void) override;

        /**
         * @copybrief BaseGripper::setPosition()
         *
         * Invokes BaseGripper::setPosition(),
         * transforms the desired gripper width to a servo angle
         * using the JOINT2ACTUATOR() macro and finally sets the
         * servo position using setServoPosition()
         * @param width width in m.
         * @return see setServoPosition()
         */
        err_type_t setPosition(float width) override;

    protected:
        /**
         * @brief Sets the servo position of the gripper actuator in degrees.
         * 
         * Calculates a PWM dutycycle for the desired servo angle and 
         * invokes RPI_PWM::setDutyCycle() to set it. 
         * @param angle in degrees.
         * @return err_type_t::OK on success, err_type_t::COMM_ERROR if the dutycycle can not be set.
         */
        err_type_t setServoPosition(float angle);

        RPI_PWM _pwm;   ///< RPI_PWM object to generate PWM voltage for servo control
        int _freq = 50; ///< PWM frequency in Hz

    private:
        
    };
}
#endif // MGRIPPER_H