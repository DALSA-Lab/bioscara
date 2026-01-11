/**
 * @file mBaseGripper.h
 * @author sbstorz
 * @brief File containing the BaseGripper class
 * @version 0.1
 * @date 2025-05-27
 *
 * @copyright Copyright (c) 2025

 * Dont include this file directly, instead use one of the derived classes.
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
     */
    class BaseGripper
    {
    public:
        /**
        * @brief Construct a new BaseGripper object
        *
        * The gripper has the reduction \f$r\f$ and offset \f$o\f$ parameters which are used to translate from a desired gripper width to the servo angle. The relationship between gripper width \f$w\f$ and acutator angle \f$\alpha\f$ is as follows:
            \f[
            \alpha = r (w-o)
            \f]

            To determine these parameters execute the following steps:

            -# Manually set the gripper to an open position by setting a actuator angle. Be carefull to not exceed the physical limits of the gripper since the actuator is strong enough to break PLA before stalling.
            -# Measure the gripper width \f$w_1\f$ and note the set actuator angle \f$\alpha_1\f$.
            -# Move the gripper to a more closed position that still allows you to accurately measure the width
            -# Measure the second width \f$w_2\f$ and note the corresponding angle \f$\alpha_2\f$
            -# Calculate the offset \f$o\f$:
            \f[
            o = \frac{\alpha_1 w_2 -  \alpha_2 w_1}{\alpha_1 - \alpha_2}
            \f]
            -# Calculate the reduction \f$r\f$:
            \f[
            r = \frac{\alpha_1}{w_1 - o}
            \f]
        *
        * @param reduction the gripper width to actuator reduction ratio
        * @param offset gripper width to actuator zero offset
        * @param min lower limit in m
        * @param max upper limit in m
        * @param backup_init_pos initial position assumed if none can be retrieved from the buffer file
        */
        BaseGripper(float reduction, float offset, float min, float max, float backup_init_pos);

        /**
         * @brief Destroy the BaseGripper object
         * 
         * invokes the disable() and deinit() method to clean up.
         */
        ~BaseGripper(void);

        /**
         * @brief Initializes the gripper
         *
         * @return err_type_t::OK
         */
        virtual err_type_t init(void);

        /**
         * @brief Deinitializes the gripper
         *
         * @return err_type_t::OK
         */
        virtual err_type_t deinit(void);

        /**
         * @brief Enables the gripper
         *
         * Since the gripper has no position feedback its
         * position is initially unknown. For this reason we try to 
         * retrieve the latest commanded position from a buffer file using 
         * retrieve_last_position(). If this fails the #_backup_init_pos is used.
         * @return err_type_t::OK
         */
        virtual err_type_t enable(void);

        /**
         * @brief Disables the gripper
         *
         * As described in enable()
         * @return err_type_t::OK
         */
        virtual err_type_t disable(void);

        /**
         * @brief Sets the gripper width in m
         *
         * Arguments outside the allowed range are bounded to #_min and #_max.
         * @param width width in m.
         * @return err_type_t::OK
         */
        virtual err_type_t setPosition(float width);

        /**
         * @brief Gets the gripper width.
         * 
         * @note Since the PWM servo has no position feedback, the latest
         * position command is returned.
         * @param width width in m.
         * @return err_type_t::OK
         */
        virtual err_type_t getPosition(float &width);

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
         * @param pos position value to store
         * @return err_type_t::OK on success, err_type_t::ERROR if writing buffer file fails.
         */
        err_type_t save_last_position(float pos);

        /**
         * @brief Retrieves the stored position from the buffer file.
         *
         * @param pos position value that is retrieved
         * @return err_type_t::OK on success, err_type_t::ERROR if parsing buffer file fails.
         */
        err_type_t retrieve_last_position(float &pos);

        float _reduction = 1;          ///< gripper width to actuator reduction ratio
        float _offset = 0;             ///< gripper width position offset
        float _min = 0;                ///< gripper width lower limit
        float _max = 0;                ///< gripper width upper limit
        float _backup_init_pos = 0.0;  ///< Initial position assumed if none can be retrieved from the buffer file
        float _pos = _backup_init_pos; ///< stored position
        float _pos_get = _pos;         ///< reported position
    private:
        std::chrono::_V2::system_clock::time_point _new_cmd_time =
            std::chrono::high_resolution_clock::now();
    };
}
#endif // MBASEGRIPPER_H