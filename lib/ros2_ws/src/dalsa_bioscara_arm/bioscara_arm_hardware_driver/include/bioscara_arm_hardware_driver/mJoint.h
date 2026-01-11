/**
 * @file mJoint.h
 * @author sbstorz
 * @brief File including the Joint class
 * @version 0.1
 * @date 2025-05-29
 *
 * @copyright Copyright (c) 2025
 *
 */

#ifndef MJOINT_H
#define MJOINT_H

#include <iostream>
#include "bioscara_arm_hardware_driver/mBaseJoint.h"
#include "bioscara_arm_hardware_driver/uTransmission.h"

namespace bioscara_hardware_drivers
{
  /**
   * @brief Representing a single hardware joint connected via I2C
   *
   */
  class Joint : public BaseJoint
  {
  public:
    /**
     * @brief Create a Joint object
     *
     * The Joint object represents a single joint and its actuator.
     * Each Joint has a transmission with the following relationship: \n
     *
     * > actuator position = (joint position - offset) * reduction \n
     * > joint position = actuator position / reduction + offset
     *
     *
     * @param name string device name for identification
     * @param address 1-byte I2C device adress (0x11 ... 0x14) for J1 ... J4
     * @param reduction gear reduction of the joint. This is used to transform position
     * and velocity values between in joint units and actuator (stepper) units.
     * The sign depends on the direction the motor is mounted and is turning. Adjust such that the joint moves in the positive
     * direction on on positive joint commands. Cable polarity has no effect since the motors
     * automatically adjust to always run in the 'right' direction from their point of view. \n
     * J1: 35 \n
     * J2: -2*pi/0.004 (4 mm linear movement per stepper revolution) \n
     * J3: 24 \n
     * J4: 12
     * @param min lower joint limit in joint units. \n
     * J1: -3.04647 \n
     * J2: -0.0016 \n
     * J3: -2.62672 \n
     * J4: -3.01069
     * @param max upper joint limit in joint units. \n
     * J1: 3.04647 \n
     * J2: 0.3380 \n
     * J3: 2.62672 \n
     * J4: 3.01069
     */
    Joint(const std::string name, const int address, const float reduction, const float min, const float max);

    /**
     * @brief Destroy the Joint object
     *
     * @copybrief BaseJoint
     */
    ~Joint(void);

    /**
     * @brief Initialize connection to a joint via I2C
     *
     * Adds the joint to the I2C bus and tests if is responsive by invoking checkCom().
     * If the joint is homed, retrieve the homing position stored on the joint by invoking
     * getHomingOffset().
     *
     * @return err_type_t
     */
    err_type_t init(void) override;

    /**
     * @brief Disconnects from a joint.
     *
     * Removes the joint from the I2C bus by invoking closeI2CDevHandle().
     *
     * @return err_type_t
     */
    err_type_t deinit(void) override;

    err_type_t enable(u_int8_t driveCurrent, u_int8_t holdCurrent) override;

    /**
     * @copybrief BaseJoint::postHoming()
     *
     * Invokes BaseJoint::postHoming() and additionally saves the homing offset to the 
     * joint controller setHomingOffset()
     *
     * @return err_type_t
     */
    err_type_t postHoming(void) override;

    err_type_t getPosition(float &pos) override;

    err_type_t setPosition(float pos) override;

    err_type_t moveSteps(int32_t steps) override;

    err_type_t getVelocity(float &vel) override;

    err_type_t setVelocity(float vel) override;

    err_type_t checkOrientation(float angle = 2.0) override;

    err_type_t stop(void) override;

    err_type_t disableCL(void) override;

    err_type_t setDriveCurrent(u_int8_t current) override;

    err_type_t setHoldCurrent(u_int8_t current) override;

    err_type_t setBrakeMode(u_int8_t mode) override;

    err_type_t setMaxAcceleration(float maxAccel) override;

    err_type_t setMaxVelocity(float maxVel) override;

    err_type_t enableStallguard(u_int8_t sensitivity) override;

    err_type_t getFlags(void) override;

  protected:

    /**
     * @brief Retrieves the homing position from the last homing.
     *
     * The homing position is stored on the joint to make it persistent as long as the joint is powered up.
     *
     * @return err_type_t
     */
    err_type_t getHomingOffset(float &offset);

    /**
     * @brief Stores the homing position on the joint.
     *
     * The homing position is stored on the joint to make it persistent as long as the joint is powered up.
     *
     * @return err_type_t
     */
    err_type_t setHomingOffset(const float offset);
    
    /**
     * @copybrief BaseJoint::_home()
     *
     * The joint will start rotating with the specified speed
     * until a resistance which drives the PID error above the specified threshold is encountered.
     * At this point the stepper stops and zeros the encoder.
     * @param velocity  signed velocity in rad/s or m/s. Must be between 1.0 < RAD2DEG(JOINT2ACTUATOR(velocity, reduction, 0)) / 6 < 250.0
     * @param sensitivity Encoder pid error threshold 0 to 255.
     * @param current homing current, determines how easy it is to stop the motor and thereby provoke a stall
     *
     * @return err_type_t
   */
    err_type_t _home(float velocity, u_int8_t sensitivity, u_int8_t current);

    /**
     * @brief Check if communication to the joint is established
     *
     * Sends a PING to and expects a ACK from the joint.
     *
     * @return err_type_t
    */
    err_type_t checkCom(void);

    float reduction = 1; ///< Joint to actuator reduction ratio
    float offset = 0;    ///< Joint position offset
    float min = 0;       ///< Joint lower limit
    float max = 0;       ///< Joint upper limit

  private:
    template <typename T>
    int read(const stp_reg_t reg, T &data, u_int8_t &flags);

    template <typename T>
    int write(const stp_reg_t reg, T data, u_int8_t &flags);

    int address;     ///< I2C adress
    int handle = -1; ///< I2C bus handle
  };
}
#include "bioscara_arm_hardware_driver/mJoint.hpp"

#endif