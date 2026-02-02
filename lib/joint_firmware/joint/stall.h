/**
 * @file stall.h
 * @author sbstorz
 * @brief Helper functions for improved stall detection
 * @version 0.1
 * @date 2025-05-27
 *
 * @copyright Copyright (c) 2025
 *
 *
 */
#ifndef STALL_H
#define STALL_H
#include <stdlib.h>

namespace bioscara_joint_firmware {
/**
  * @brief computes the speed adaptive threshold.
  *
  * @param qd_rad speed in rad/s. Should be measured speed because set speed
  * is only available in velocity mode.
  */
float stall_threshold(float qd_rad, float offset) {
  /* y = ax + b */
  float a = STALL_SLOPE;
  float b = offset;
  if ((abs(qd_rad) >= STALL_WINDOW_B1) && (abs(qd_rad) <= STALL_WINDOW_B2)) {
    b += STALL_WINDOW_OFFSET;
  }
  return a * qd_rad + b;
}
}
#endif
