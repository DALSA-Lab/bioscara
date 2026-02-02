/**
 * @file filters.h
 * @author sbstorz
 * @brief Helper classes for FIR and IIR filters
 * @version 0.1
 * @date 2025-05-27
 *
 * @copyright Copyright (c) 2025
 *
 * This file contains time series filters that can be used to filter data.
 *
 */
#include <stdlib.h>

namespace bioscara_joint_firmware {

/**
 * @brief Simple discrete IIR lowpass filter
 *
 */
class Lowpass {

protected:
  float K;    ///< Filter gain
  float Ts;   ///< Filter sampling time
  float tau;  ///< Filter timeconstant
  float x;    ///< Filter state

public:

  /**
  * @brief Constucts and initializes the Lowpass filter
  * @param gain
  * @param sampleTime
  * @param timeconstant
  * 
  */
  Lowpass(float gain = 1, float sampleTime = 0.1, float timeconstant = 1.0) {
    this->K = gain;
    this->Ts = sampleTime;
    this->tau = timeconstant;
    x = 0.0;
  }

  /**
  * @brief Update the filter state. Must be called with the constant period #Ts
  * @return The updated filter state
  */
  float updateState(float u) {
    x = (1 - (Ts / tau)) * x + K * (Ts / tau) * u;
    return x;
  }

  /**
  * @brief Resets the filter state #x to 0.0
  *
  */
  void resetState(void) {
    x = 0.0;
  }
};

/**
 * @brief Simple FIR moving maximum filter
 *
 * Implements a circular ringbuffer holding the sampled data over the window size #M
 */
class MovMax {

protected:
  unsigned int M = 200;  ///< Window Size

  float *cb_data;        ///<  Pointer to ring buffer holding the data
  unsigned int cb_index; ///< Current ringbuffer index


public:
  /**
   * @brief Constucts and initializes the MovMax filter
   *
   * Allocates a ring buffer #cb_data of size #M.
   * @param windowSize the window size
   * 
   */
  MovMax(float windowSize)
    : M(windowSize), cb_index(0), cb_data(0) {

    cb_data = (float *)malloc(windowSize * sizeof(float));  // allocate memory for buffer
  }

  /**
  * @brief Update and return the filter state.
  * @return The updated filter state
  */
  float updateState(float u) {

    cb_data[cb_index] = u;
    cb_index = (cb_index + 1) % M;


    float max = 0;
    for (size_t i = 0; i < M; i++) {
      if (cb_data[i] > max) {
        max = cb_data[i];
      }
    }

    return max;
  }
};
}
