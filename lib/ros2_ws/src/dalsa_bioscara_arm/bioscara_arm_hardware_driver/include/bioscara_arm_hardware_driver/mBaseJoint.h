/**
 * @file mBaseJoint.h
 * @author sbstorz
 * @brief File including the BaseJoint class
 * @version 0.1
 * @date 2025-05-29
 *
 * @copyright Copyright (c) 2025
 *
 */

#ifndef MBASEJOINT_H
#define MBASEJOINT_H

#include <iostream>
#include "bioscara_arm_hardware_driver/uErr.h"

namespace bioscara_hardware_drivers
{
  /**
   * @brief Generic base class to control a single joint.
   *
   * This class is a wrapper function to interact with a robot joint
   * either through a MockJoint or hardware Joint object.
   */
  class BaseJoint
  {
  public:
    /**
     *
     * @brief register and command definitions
     *
     * a register can be read (R) or written (W), each register has a size in bytes.
     * The payload can be split into multiple values or just be a single value.
     * Note that not all functions are implemented.
     *
     */
    enum stp_reg_t
    {
      NONE = 0x00,                ///< Used for signalling purposes
      PING = 0x0f,                ///< R; Size: 1; [(char) ACK]
      SETUP = 0x10,               ///< W; Size: 2; [(uint8) holdCurrent, (uint8) driveCurrent]
      SETRPM = 0x11,              ///< W; Size: 4; [(float) RPM]
      GETDRIVERRPM = 0x12,        ///<
      MOVESTEPS = 0x13,           ///< W; Size: 4; [(int32) steps]
      MOVEANGLE = 0x14,           ///<
      MOVETOANGLE = 0x15,         ///< W; Size: 4; [(float) degrees]
      GETMOTORSTATE = 0x16,       ///<
      RUNCOTINOUS = 0x17,         ///<
      ANGLEMOVED = 0x18,          ///< R; Size: 4; [(float) degrees]
      SETCURRENT = 0x19,          ///< W; Size: 1; [(uint8) driveCurrent]
      SETHOLDCURRENT = 0x1A,      ///< W; Size: 1; [(uint8) holdCurrent]
      SETMAXACCELERATION = 0x1B,  ///<
      SETMAXDECELERATION = 0x1C,  ///<
      SETMAXVELOCITY = 0x1D,      ///<
      ENABLESTALLGUARD = 0x1E,    ///< W; Size: 1; [(uint8) threshold]
      DISABLESTALLGUARD = 0x1F,   ///<
      CLEARSTALL = 0x20,          ///<
      SETBRAKEMODE = 0x22,        ///< W; Size: 1; [(uint8) mode]
      ENABLEPID = 0x23,           ///<
      DISABLEPID = 0x24,          ///<
      ENABLECLOSEDLOOP = 0x25,    ///<
      DISABLECLOSEDLOOP = 0x26,   ///< W; Size: 1; [(uint8) 0]
      SETCONTROLTHRESHOLD = 0x27, ///<
      MOVETOEND = 0x28,           ///<
      STOP = 0x29,                ///< W; Size: 1; [(uint8) mode]
      GETPIDERROR = 0x2A,         ///<
      CHECKORIENTATION = 0x2B,    ///< W; Size: 4; [(float) degrees]
      GETENCODERRPM = 0x2C,       ///< R; Size: 4; [(float) RPM]
      HOME = 0x2D,                ///< W; Size: 4; [(uint8) current, (int8) sensitivity, (uint8) speed, (uint8) direction]
      HOMEOFFSET = 0x2E,          ///< R/W; Size: 4; [(float) -]
    };

    /**
     * @brief Create a Joint object
     *
     * The Joint object represents a single joint.
     *
     * @param name string device name for logging.
     */
    BaseJoint(const std::string name);

    /**
     * @brief Destroy the BaseJoint object
     *
     * Invokes the disable() and deinit() method to clean up.
     */
    ~BaseJoint(void);

    /**
     * @brief Initialize the joint communication
     *
     * @return err_type_t
     *
     */
    virtual err_type_t init(void);

    /**
     * @brief Denitialize the joint communication
     *
     * @return err_type_t
     *
     */
    virtual err_type_t deinit(void);

    /**
     * @brief Engages the joint.
     *
     * This function prepares the motor for movement. After successfull execution the joint
     * is ready to accept setPosition() and setVelocity() commands.
     *
     * The function sets the drive and hold current for the specified joint and engages the motor.
     * The currents are in percent of driver max. output (2.5A, check with TMC5130 datasheet or Ustepper documentation)
     * @param driveCurrent drive current in 0-100 % of 2.5A output (check uStepper doc.)
     * @param holdCurrent hold current in 0-100 % of 2.5A output (check uStepper doc.)
     * @return err_type_t
     */
    virtual err_type_t enable(u_int8_t driveCurrent, u_int8_t holdCurrent);

    /**
     * @brief disenganges the joint
     *
     * invokes stop(), sets hold and drive current to 0
     * and sets the the joint brake mode to freewheeling
     * @return err_type_t
     */
    virtual err_type_t disable(void);

    /**
     * @brief Blocking implementation to home the joint
     *
     * Homing the joint is neccessary after its controller has been powered off.
     * The joint can be moved while the encoders are inactive and hence the position
     * at startup is unknow. See _home() for information on implementation and
     * description of the paramters.
     *
     * This is a blocking implementation which only returns after the the joint is no longer busy.
     * First startHoming() is called, and subsequently waits with wait_while_busy() to finish homing.
     * Lastly postHoming() is called.
     * @return err_type_t
     */
    virtual err_type_t home(float velocity, u_int8_t sensitivity, u_int8_t current);

    /**
     * @brief non-blocking implementation to home the joint
     *
     * Homing the joint is neccessary after its controller has been powered off.
     * The joint can be moved while the encoders are inactive and hence the position
     * at startup is unknow. See _home() for information on implementation and
     * description of the paramters.
     *
     *
     * This method returns immediatly after starting the homing sequence and
     * should be used when the blocking implementation is not acceptable,
     * for example in a realtime loop.
     *
     * This method sets the #current_b_cmd flag to stp_reg_t::HOME
     * @return err_type_t
     */
    virtual err_type_t startHoming(float velocity, u_int8_t sensitivity, u_int8_t current);

    /**
     * @brief perform tasks after a non-blocking homing
     *
     * This method resets the #current_b_cmd to stp_reg_t::NONE, checks if the joint is homed,
     * and saves the homing offset to the joint.
     *
     * @return err_type_t
     */
    virtual err_type_t postHoming(void);

    /**
     * @brief get the current joint position in radians or m for
     * cylindrical and prismatic joints respectively.
     *
     * @warning If the joint is not homed this method does not return an error.
     * Instead `pos` will be 0.0.
     *
     * @param pos the current joint position in rad or m.
     * @return err_type_t
     */
    virtual err_type_t getPosition(float &pos) = 0;

    /**
     * @brief get the current joint position in radians or m for
     * cylindrical and prismatic joints respectively.
     *
     * @param pos the commanded joint position in rad or m.
     * @return err_type_t
     */
    virtual err_type_t setPosition(float pos);

    /**
     * @brief Move full steps.
     *
     * This function can be called even when not homed.
     *
     * @param steps number of full steps
     * @return err_type_t
     */
    virtual err_type_t moveSteps(int32_t steps);

    /**
     * @brief get the current joint velocity in radians/s or m/s for
     * cylindrical and prismatic joints respectively.
     *
     * @param vel the current joint velocity in rad/s or m/s.
     * @return err_type_t
     */
    virtual err_type_t getVelocity(float &vel) = 0;

    /**
     * @brief Set the current joint velocity in radians/s or m/s for
     * cylindrical and prismatic joints respectively.
     *
     * @param vel the commanded joint velocity in rad/s or m/s.
     * @return err_type_t
     */
    virtual err_type_t setVelocity(float vel);

    /**
     * @brief Calls the checkOrientation method of the motor. Checks in which direction the motor is turning.
     *
     * @note This is a blocking function.
     * @param angle degrees how much the motor should turn. A few degrees is sufficient.
     * @return err_type_t
     */
    virtual err_type_t checkOrientation(float angle = 2.0);

    /**
     * @brief Stops the motor.
     *
     * Stops the motor by setting the maximum velocity to zero and the position setpoint
     * to the current position
     *
     * @return err_type_t
     */
    virtual err_type_t stop(void);

    virtual err_type_t disableCL(void);

    /**
     * @brief Set the Drive Current.
     * @param current 0% - 100% of driver current
     * @return err_type_t
     */
    virtual err_type_t setDriveCurrent(u_int8_t current);

    /**
     * @brief Set the Hold Current.
     * @param current 0% - 100% of driver current
     * @return err_type_t
     * -1 on communication error,
     */
    virtual err_type_t setHoldCurrent(u_int8_t current);

    /**
     * @brief Set Brake Mode.
     * @param mode Freewheel: 0, Coolbrake: 1, Hardbrake: 2
     * @return err_type_t
     */
    virtual err_type_t setBrakeMode(u_int8_t mode);

    /**
     * @brief Set the maximum permitted joint acceleration (and deceleration) in rad/s^2 or m/s^2 for cylindrical
     * and prismatic joints respectively.
     *
     * @param maxAccel maximum joint acceleration.
     * @return err_type_t
     */
    virtual err_type_t setMaxAcceleration(float maxAccel);

    /**
     * @brief Set the maximum permitted joint velocity in rad/s or m/s for cylindrical
     * and prismatic joints respectively.
     *
     * @param maxVel maximum joint velocity.
     * @return err_type_t
     */
    virtual err_type_t setMaxVelocity(float maxVel);

    /**
     * @brief Enable encoder stall detection of the joint.
     *
     * If the PID error exceeds the set threshold a stall is triggered and the motor disabled.
     * A detected stall can be reset by homing or reenabling the joint using enable().
     * @note If stall detection shall be enabled, invoke this method  AFTER enabling the joint with enable().
     * @param sensitivity value of threshold. 0 - 255 where lower is more sensitive.
     * @return err_type_t
     */
    virtual err_type_t enableStallguard(u_int8_t sensitivity);

    /**
     * @brief Checks the state if the motor is homed.
     *
     * Reads the internal state flags from the last transmission. If an update is neccessary call getFlags() before invoking this function.
     *
     * @return true if the motor is homed,
     * false if not.
     */
    virtual bool isHomed(void);

    /**
     * @brief Checks the state if the motor is enabled.
     *
     * Reads the internal state flags from the last transmission. If an update is neccessary call getFlags() before invoking this function.
     * If the motor actually can move depends on the state of the STALLED flag which can be checked using Joint::isStalled().
     *
     * @return true if the motor is enabled,
     * false if not.
     */
    virtual bool isEnabled(void);

    /**
     * @brief Checks if the motor is stalled.
     *
     * Reads the internal state flags from the last transmission. If an update is neccessary call getFlags() before invoking this function.
     * @return true if the motor is stalled,
     * false if not.
     */
    virtual bool isStalled(void);

    /**
     * @brief Checks if the joint controller is busy processing a blocking command.
     *
     * Reads the internal state flags from the last transmission. If an update is neccessary call getFlags() before invoking this function.
     * @return true if a blocking command is currently executing,
     * false if not.
     */
    virtual bool isBusy(void);

    /**
     * @brief get the latest driver state flags from the joint
     * @param flags if succesfull, populated with the latest flags
     * @return err_type_t
     */
    virtual err_type_t getFlags(u_int8_t &flags);

    /**
     * @brief Overload of getFlags(u_int8_t &flags)
     * @return err_type_t
     */
    virtual err_type_t getFlags(void);

    /**
     * @brief get the currently active blocking command
     *
     * @return The the command of type stp_reg_t
     */
    virtual stp_reg_t getCurrentBCmd(void);

    std::string name; ///< Joint name for logging

  protected:
    /**
     * @brief Blocking loop waiting for BUSY flag to reset.
     *
     * @param period_ms time in ms between polls.
     */
    virtual void wait_while_busy(const float period_ms);

    /**
     * @brief Call to start the homing sequence of a joint.
     */
    virtual err_type_t _home(float velocity, u_int8_t sensitivity, u_int8_t current) = 0;

    /**
     * @brief State flags transmitted with every I2C transaction.
     *
     * The transmission flags purpose are to transmit the joints current state.
     * Note: They can not be used as error indication of the execution of a transmitted write command,
     * since commands are executed after the I2C transaction is completed. The status flags are one
     * byte with following structure: \n
     *
     * |BIT7|BIT6|BIT5|BIT4|BIT3|BIT2|BIT1|BIT0|
     * | ---- | ---- | ---- | ---- | ---- | ---- | ---- | ---- |
     * |reserved|reserved|reserved|reserved|NOTENABLED|NOTHOMED|BUSY|STALL|
     *
     * \b STALL is set if a stall from the stall detection is sensed and the joint is stopped.
     * The flag is cleared when the joint is homed or the Stallguard enabled. \n
     * \b BUSY is set if the slave is busy processing a previous command. \n
     * \b NOTHOMED is cleared if the joint is homed. Movement is only allowed if this flag is clear \n
     * \b NOTENABLED is cleared if the joint is enabled after calling enable()
     */
    u_int8_t flags = 0b00001100;

    stp_reg_t current_b_cmd = NONE; ///< Keeps track if a blocking command is being executed

  private:
  };
}
#endif