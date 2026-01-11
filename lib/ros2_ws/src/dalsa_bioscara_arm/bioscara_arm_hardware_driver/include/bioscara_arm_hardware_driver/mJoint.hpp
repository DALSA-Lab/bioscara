/**
 * @file mJoint.hpp
 * @author sbstorz
 * @brief Templated functions for the Joint class.
 * @version 0.1
 * @date 2025-05-29
 *
 * @copyright Copyright (c) 2025
 *
 * This header must be included at the END of the mJoint.h file.
 */
#include "bioscara_arm_hardware_driver/mJoint.h"
#include "bioscara_arm_hardware_driver/uI2C.h"
#include "bioscara_arm_hardware_driver/common.h"

namespace bioscara_hardware_drivers
{
    /**
     * @brief Wrapper function to request data from the I2C slave.
     *
     * Allocates a buffer of size sizeof(T) + RFLAGS_SIZE.
     * Invokes readFromI2CDev(), and copies the received payload to \a data  and the transmisison flags
     * to \a flags. See Joint::flags for details.

     * @tparam T Datatype of value to be transmitted
     * @param reg stp_reg_t register to read
     * @param data reference to store payload.
     * @param flags reference to a byte which stores the return flags
     * @return 0 on OK, negative on error
     */
    template <typename T>
    int Joint::read(const stp_reg_t reg, T &data, u_int8_t &flags)
    {
        size_t size = sizeof(T) + RFLAGS_SIZE;
        char buf[MAX_BUFFER + RFLAGS_SIZE];
        int n = readFromI2CDev(this->handle, reg, buf, size);
        if (n != static_cast<int>(size))
        {
            return -1;
        }
        memcpy(&data, buf, size - RFLAGS_SIZE);
        memcpy(&flags, buf + size - RFLAGS_SIZE, RFLAGS_SIZE);
        return 0;
    }

    /**
     * @brief Wrapper function to send command to the I2C slave.
     *
     * Allocates a buffer of size sizeof(T) + RFLAGS_SIZE. Copyies \a data to the buffer
     * and invokes writeToI2CDev(). The flags received from the transaction are copied to \a flags.
     * The flags are described in Joint::read().
     *
     *
     * @tparam T Datatype of value to be transmitted
     * @param reg stp_reg_t command to execute
     * @param data payload to transmit. It is the users responsibility to populate the right
     * amount of data for the relevant register
     * @param flags reference to a byte which stores the return flags
     * @return 0 on OK, negative on error
     */
    template <typename T>
    int Joint::write(const stp_reg_t reg, T data, u_int8_t &flags)
    {
        size_t size = sizeof(T) + RFLAGS_SIZE;
        char buf[MAX_BUFFER + RFLAGS_SIZE];
        memcpy(buf, &data, size - RFLAGS_SIZE);
        int rc = writeToI2CDev(this->handle, reg, buf, size - RFLAGS_SIZE, buf + size - RFLAGS_SIZE);
        rc = rc > 0 ? 0 : rc;
        memcpy(&flags, buf + size - RFLAGS_SIZE, RFLAGS_SIZE);
        return rc;
    }
}