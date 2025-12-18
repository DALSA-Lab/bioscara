/**
 * @file joint.h
 * @author sbstorz
 * @brief joint firmware header
 * @version 0.1
 * @date 2025-05-27
 *
 * @copyright Copyright (c) 2025
 *
 * This file contains definitions and macros for the joint firmware.
 *
 */

#ifndef JOINT_H
#define JOINT_H
#include <Arduino.h>

#define ACK 'O'
#define NACK 'N'

/**
 * @brief Maximum size of I2C Payload in bytes
 *
 * 4 bytes used to transmit floats and int32_t
 */
#define MAX_BUFFER 4 // Bytes

/**
 * @brief Size of the return flags in bytes
 *
 * Only one byte used and hence set to 1.
 */
#define RFLAGS_SIZE 1

/**
 * @brief Macro to dump a buffer to the serial console.
 *
 * @param buffer pointer to a buffer to dump to the console
 * @param size number of bytes to dump
 */
#define DUMP_BUFFER(buffer, size)     \
  {                                   \
    Serial.print("Buffer dump: ");    \
    for (size_t i = 0; i < size; i++) \
    {                                 \
      Serial.print(buffer[i], HEX);   \
      Serial.print(" ");              \
    }                                 \
    Serial.println();                 \
  }

/**
 * @brief register and command definitions
 *
 * a register can be read (R) or written (W), each register has a size in bytes.
 * The payload can be split into multiple values or just be a single value.
 * Note that not all functions are implemented.
 *
 */
enum stp_reg_t
{
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
  SETMAXACCELERATION = 0x1B,  ///< W; Size: 4; [(float) deg/s^2]
  SETMAXDECELERATION = 0x1C,  ///<
  SETMAXVELOCITY = 0x1D,      ///< W; Size: 4; [(float) deg/s]
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
  HOME = 0x2D,                ///< W; Size: 4; [(uint8) current, (uint8) sensitivity, (uint8) speed, (uint8) direction]
  HOMEOFFSET = 0x2E,          ///< R/W; Size: 4; [(float) -]
};

/**
 * @brief Reads a value from a buffer to a value of the specified type
 * @param val Reference to output variable
 * @param rxBuf Buffer to read value from
 * @param rx_length Length of the buffer
 */
template <typename T>
void readValue(T &val, uint8_t *rxBuf, size_t rx_length)
{
  memcpy(&val, rxBuf, rx_length);
}

/**
 * @brief Writes a value of the specified type to a buffer.
 * @param val Reference to input variable
 * @param txBuf pointer to tx buffer
 * @param tx_length Length of the buffer returne
 * @return 0 On success
 */
template <typename T>
int writeValue(const T val, uint8_t *txBuf, size_t &tx_length)
{
  tx_length = sizeof(T);
  memcpy(txBuf, &val, tx_length);
  return 0;
}

#endif
