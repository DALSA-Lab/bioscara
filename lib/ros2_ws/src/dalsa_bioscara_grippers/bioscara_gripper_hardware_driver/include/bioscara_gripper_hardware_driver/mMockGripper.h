/**
 * @file mMockGripper.h
 * @author sbstorz
 * @brief File containing the MockGripper class
 * @version 0.1
 * @date 2025-05-27
 *
 * @copyright Copyright (c) 2025

 * Include this file for API functions to interact with the MockGripper.
 *
 */
#ifndef MMOCKGRIPPER_H
#define MMOCKGRIPPER_H
#include "bioscara_gripper_hardware_driver/mBaseGripper.h"
#include "bioscara_arm_hardware_driver/uErr.h"

namespace bioscara_hardware_drivers
{
    /**
     * @brief Child class for to mock the gripper hardware.
     *
     * Use this class if you dont wish to use the gripper hardware. Commands are simply mirrored.
     *
     */
    class MockGripper : public BaseGripper
    {
    public:
        /**
         * @brief Construct a new MockGripper object
         *
         * see the BaseGripper constructor for a description of the parameters.
         */
        MockGripper(float reduction, float offset, float min, float max, float backup_init_pos);

    protected:
    private:
    };
}
#endif // MMOCKGRIPPER_H