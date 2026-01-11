/**
 * @file uErr.h
 * @author sbstorz
 * @brief Defining common return types
 * @version 0.1
 * @date 2025-11-05
 *
 * @copyright Copyright (c) 2025
 *
 */
#ifndef UERR_H
#define UERR_H

#include <string>

namespace bioscara_hardware_drivers
{
    /**
     * @brief Enum defining common error types
     * 
     */
    enum class err_type_t
    {
        OK = 0, ///< Success
        ERROR = -1, ///< Generic Error
        NOT_HOMED = -2, ///< Joint not homed or failed to home
        NOT_ENABLED = -3, ///< Joint not enabled or failed to enable
        STALLED = -4, ///< Joint stalled
        NOT_INIT = -5, ///< Joint not initialized
        COMM_ERROR = -6, ///< Communication error
        INVALID_ARGUMENT = -101, ///< Argument violates conditions
        INCORRECT_STATE = -109, ///< Joint is busy executing another blocking command

    };

    /**
     * @brief Converts an error code to a string and returns it
     * 
     * @param err 
     * @return std::string 
     */
    std::string error_to_string(err_type_t err);
}

/**
 * @brief Macro which executes a function and returns from the calling function with the error code if the called function fails.
 *
 * Adapted from the [ESP-IDF](https://github.com/espressif/esp-idf/blob/ff97953b32a32e44f507593320b50d728eea3f06/components/esp_common/include/esp_check.h#L19)
 * @param x function to call
 *
 */
#define RETURN_ON_ERROR(x)                                       \
    do                                                           \
    {                                                            \
        bioscara_hardware_drivers::err_type_t err_rc_ = (x);      \
        if (err_rc_ != bioscara_hardware_drivers::err_type_t::OK) \
        {                                                        \
            return err_rc_;                                      \
        }                                                        \
    } while (0);

/**
 * @brief Macro which returns the calling function with specified error_code if the given condition is false.
 *
 * Adapted from the [ESP-IDF](https://github.com/espressif/esp-idf/blob/ff97953b32a32e44f507593320b50d728eea3f06/components/esp_common/include/esp_check.h#L90)
 * @param a expression that evaluates to true or false
 * @param err_code return code to return on false
 */
#define RETURN_ON_FALSE(a, err_code) \
    do                               \
    {                                \
        if (!(a))                    \
        {                            \
            return err_code;         \
        }                            \
    } while (0);

/**
 * @brief Macro which returns the calling function with specified error_code if the given condition is negative.
 *
 * Adapted from the [ESP-IDF](https://github.com/espressif/esp-idf/blob/ff97953b32a32e44f507593320b50d728eea3f06/components/esp_common/include/esp_check.h#L90)
 * @param a expression that evaluates to a signed number
 * @param err_code return code to return on false
 */
#define RETURN_ON_NEGATIVE(a, err_code) \
    do                                  \
    {                                   \
        if ((a) < 0)                    \
        {                               \
            return err_code;            \
        }                               \
    } while (0);

#endif // UERR_H