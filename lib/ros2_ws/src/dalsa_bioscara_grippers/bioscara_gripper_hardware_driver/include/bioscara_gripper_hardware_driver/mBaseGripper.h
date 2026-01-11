/**
 * @file mBaseGripper.h
 * @author sbstorz
 * @brief File containing the BaseGripper class
 * @version 0.1
 * @date 2025-05-27
 *
 * @copyright Copyright (c) 2025

 * Dont include this file directly, instead use one of the derived classes.
 *
 */
#ifndef MBASEGRIPPER_H
#define MBASEGRIPPER_H
#include "bioscara_arm_hardware_driver/uErr.h"
#include <chrono>

namespace bioscara_hardware_drivers
{

    /**
     * @brief Generic base class for gripper control implementations
     *
     * This class is a wrapper function to interact with the robot gripper either through a MockGripper
     * or hardware Gripper object.
     *
     */
    class BaseGripper
    {
    public:
        /**
         * @brief Construct a new BaseGripper object
         *
         * ## Calculating reduction and offset
         * The gripper has the reduction \fr\f and offset \fo\f parameters which are used to translate from a desired gripper width to the servo angle. The relationship between gripper width $w$ and acutator angle $\alpha$ is as follows:
            \f[
            \alpha = r (w-o)
            \f]

            To determine these parameters execute the following steps:

            1. Manually set the gripper to an open position by setting a actuator angle. Be carefull to not exceed the physical limits of the gripper since the actuator is strong enough to break PLA before stalling.
            2. Measure the gripper width \f$w_1\f$ and note the set actuator angle \f$\alpha_1\f$.
            3. Move the gripper to a more closed position that still allows you to accurately measure the width
            4. Measure the second width \f$w_2\f$ and note the corresponding angle \f$\alpha_2\f$
            5. Calculate the offset \f$o\f$:
            \f[
            o = \frac{\alpha_1 w_2 -  \alpha_2 w_1}{\alpha_1 - \alpha_2}
            \f]
            6. Calculate the reduction \f$r\f$:
            \f[
            r = \frac{\alpha_1}{w_1 - o}
            \f]            
         *
         * @param reduction the gripper width to actuator reduction ratio
         * @param offset gripper width to actuator zero offset
         * @param min lower limit
         * @param max upper limit
         * @param backup_init_pos initial position assumed if none can be retrieved from the buffer file
         */
        BaseGripper(float reduction, float offset, float min, float max, float backup_init_pos);

        ~BaseGripper(void);

        /**
         * @brief Placeholder, does nothing
         *
         * @return 0
         */
        virtual err_type_t init(void);

        /**
         * @brief Placeholder, does nothing
         *
         * @return 0
         */
        virtual err_type_t deinit(void);

        /**
         * @brief Prepares the servo for use.
         *
         * @return non-zero error code.
         */
        virtual err_type_t enable(void);

        /**
         * @brief Disables the servo.
         *
         * @return non-zero error code.
         */
        virtual err_type_t disable(void);

        /**
         * @brief Sets the gripper width in m from the closed position.
         *
         * Arguments outside the allowed range are bounded to limit min and max.
         * @param width width in m.
         */
        virtual err_type_t setPosition(float width);

        /**
         * @brief Gets the gripper as by the last command.
         *
         * @param width width in m.
         */
        virtual err_type_t getPosition(float &width);

        /**
         * @brief Sets the servo position of the gripper actuator in degrees.
         *
         * @param angle in degrees.
         */
        virtual err_type_t setServoPosition(float angle);

        /**
         * @brief Manually set reduction
         *
         * @param reduction
         */
        virtual void setReduction(float reduction);

        /**
         * @brief Manually set offset
         */
        virtual void setOffset(float offset);

    protected:
        /**
         * @brief Stores the latest position to the buffer file.
         *
         * @param pos
         * @return err_type_t
         */
        err_type_t save_last_position(float pos);

        /**
         * @brief Retrieves the stored position from the buffer file.
         *
         * @param pos
         * @return err_type_t
         */
        err_type_t retrieve_last_position(float &pos);

        float _reduction = 1;          ///< Joint to actuator reduction ratio
        float _offset = 0;             ///< Joint position offset
        float _min = 0;                ///< Joint lower limit
        float _max = 0;                ///< Joint upper limit
        float _backup_init_pos = 0.0;  ///< Initial position used if none can be retrieved from the buffer file
        float _pos = _backup_init_pos; ///< stored position
        float _pos_get = _pos;         ///< reported position
    private:
        std::chrono::_V2::system_clock::time_point new_cmd_time =
            std::chrono::high_resolution_clock::now();
    };
}
#endif // MBASEGRIPPER_H