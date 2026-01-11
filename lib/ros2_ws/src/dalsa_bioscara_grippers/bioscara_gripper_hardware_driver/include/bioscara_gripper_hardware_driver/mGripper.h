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
         * @brief Constructor of the hardware Gripper object.
         *
         * The gripper width in m is converted to a PWM dutycyle via the JOINT2ACTUATOR macro.
         *
         * @param reduction
         * @param offset
         * @param min minimum width in m.
         * @param max maxmimum width in m.
         * @param backup_init_pos initial position the gripper assumes
         *  if it can not be read from the buffer file.
         */
        Gripper(float reduction, float offset, float min, float max, float backup_init_pos);

        /**
         * @brief Prepares the servo for use.
         *
         * Starts the PWM generation but does not set a position. Must be called before a position is set.
         * The PWM pin is GPIO18. PWM chip is 0, channel 0.
         *
         * @return return code of bioscara_hardware_drivers::esp_err_t type
         */
        err_type_t enable(void) override;

        /**
         * @brief Disables the servo.
         *
         * Stops the servo and disables the PWM generation.
         *
         * @return return code of bioscara_hardware_drivers::esp_err_t type.
         */
        err_type_t disable(void) override;

        err_type_t setPosition(float width) override;

        err_type_t setServoPosition(float angle) override;

        /**
         * @brief Manually set reduction
         *
         * @param reduction
         */
        void setReduction(float reduction);

        /**
         * @brief Manually set offset
         */
        void setOffset(float offset);

    protected:
    private:
        RPI_PWM _pwm;
        int _freq = 50;
    };
}
#endif // MGRIPPER_H