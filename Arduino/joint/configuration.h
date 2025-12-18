/**
 * @file configuration.h
 * @author sbstorz
 * @brief Configuration definitions for Joint 1 to Joint 4
 * @version 0.1
 * @date 2025-05-27
 *
 * @copyright Copyright (c) 2025
 *
 * This file shall be included AFTER one of J1, J2, J3 or J4 have been defined.
 */


#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#if defined(J1)
/** Test C documentation. */
#define ADR 0x11
#define MAXACCEL 100
#define MAXVEL 10
#define STALL_WINDOW_B1 12
#define STALL_WINDOW_B2 450
#define STALL_WINDOW_OFFSET 90.0
#define STALL_SLOPE 0.0

#elif defined(J2)
#define ADR 0x12
#define MAXACCEL 100
#define MAXVEL 10
#define STALL_WINDOW_B1 12
#define STALL_WINDOW_B2 450
#define STALL_WINDOW_OFFSET 90.0
#define STALL_SLOPE 0.0

#elif defined(J3)
#define ADR 0x13
#define MAXACCEL 100
#define MAXVEL 10
#define STALL_WINDOW_B1 12
#define STALL_WINDOW_B2 450
#define STALL_WINDOW_OFFSET 90.0
#define STALL_SLOPE 0.0

#elif defined(J4)
#define ADR 0x14
#define MAXACCEL 100
#define MAXVEL 10
#define STALL_WINDOW_B1 12
#define STALL_WINDOW_B2 450
#define STALL_WINDOW_OFFSET 90.0
#define STALL_SLOPE 0.0
#else

/* Below only defined for documentation */
/**
 * @brief I2C adress of joint n is 0x1n.
 */
#define ADR 0x11

/**
 * @brief Maximum acceleration in steps/s^2. Can be set for each joint depending on inertia. 
 * If set to high stalls might trigger since PID error grows too large.
 */
#define MAXACCEL 100

/**
 * @brief Maximum velocity in steps/s. Can be set for each joint.
 * If set to high stalls might trigger since PID error grows too large.
 */
#define MAXVEL 10
#error "No Joint has been defined. Define one of 'JX' where X 1,2,3,4"
#endif

#endif
