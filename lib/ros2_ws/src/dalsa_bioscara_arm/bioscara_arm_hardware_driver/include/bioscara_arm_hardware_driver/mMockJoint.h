/**
 * @file mMockJoint.h
 * @author sbstorz
 * @brief File including the MockJoint class
 * @version 0.1
 * @date 2025-05-29
 *
 * @copyright Copyright (c) 2025
 *
 */

#ifndef MMOCKJOINT_H
#define MMOCKJOINT_H

#include <iostream>
#include "bioscara_arm_hardware_driver/mBaseJoint.h"
#include <chrono>

namespace bioscara_hardware_drivers
{
  /**
   * @brief Representing a single joint mocking the hardware joint
   *
   * This mock hardware implementation simply mirrors (potentially delayed) commands.
   *
   * @note Much of this implementation is very basic not well written.
   */
  class MockJoint : public BaseJoint
  {
  public:
    MockJoint(const std::string name);

    err_type_t enable(u_int8_t driveCurrent, u_int8_t holdCurrent) override;

    err_type_t disable(void) override;

    err_type_t getPosition(float &pos) override;

    err_type_t setPosition(float pos) override;

    err_type_t getVelocity(float &vel) override;

    err_type_t setVelocity(float vel) override;

    err_type_t checkOrientation(float angle = 2.0) override;

    err_type_t stop(void) override;

    err_type_t getFlags(void) override;

    bool isHomed(void) override;

  protected:
    err_type_t _home(float velocity, u_int8_t sensitivity, u_int8_t current);

  private:
    float q = 0.0;  ///< position
    float qd = 0.0; ///< velocity

    std::chrono::_V2::system_clock::time_point last_set_position = std::chrono::high_resolution_clock::now();
    std::chrono::_V2::system_clock::time_point last_set_velocity = last_set_position;
    std::chrono::_V2::system_clock::time_point async_start_time = last_set_position;

    /**
     * @brief Compute the time since the given time point.
     *
     * @param last_call the time point to calculate the duration
     * @param update If true compute the duration and also reset \a last_call to current time
     * @return elapsed time in seconds since \a last_call
     */
    float getDeltaT(std::chrono::_V2::system_clock::time_point &last_call, bool update = true);

    stp_reg_t op_mode = NONE;
  };
}
#endif