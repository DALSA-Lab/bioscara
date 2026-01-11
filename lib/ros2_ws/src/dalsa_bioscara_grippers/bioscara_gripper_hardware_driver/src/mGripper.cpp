#include "bioscara_gripper_hardware_driver/mGripper.h"
#include "bioscara_arm_hardware_driver/uTransmission.h"

namespace bioscara_hardware_drivers
{

    Gripper::Gripper(float reduction, float offset, float min, float max, float backup_init_pos) : BaseGripper(reduction, offset, min, max, backup_init_pos)
    {
    }

    err_type_t Gripper::enable(void)
    {
        // Call parent enable to try to retrieve last position setpoint.
        BaseGripper::enable();

        // Try 3 times to enable PWM
        for (size_t i = 0; i < 3; i++)
        {
            if (_pwm.start(0, _freq, 0, 0) >= 0)
            {
                // ignore failure on setPosition
                setPosition(_pos);
                return err_type_t::OK;
            }
            usleep(10000);
        }
        return err_type_t::COMM_ERROR;
    }

    err_type_t Gripper::disable(void)
    {
        // Call parent disable to save last position.
        BaseGripper::disable();
        _pwm.stop();
        return err_type_t::OK;
    }

    err_type_t Gripper::setPosition(float width)
    {
        // Call parent to saturate input and save _pos
        BaseGripper::setPosition(width);

        float angle = JOINT2ACTUATOR(_pos, _reduction, _offset);
        return setServoPosition(angle);
    }

    err_type_t Gripper::setServoPosition(float angle)
    {
        float ton_us = angle / 90.0 * 500.0 + 1500.0;         // Ontime [us]
        float dc = ton_us / (1000 * 1000) * _freq * 100; // dutycycle [%] = ontime [s] /period [s] * 100 %
        for (size_t i = 0; i < 3; i++)
        {
            if (_pwm.setDutyCycle(dc) >= 0)
            {
                return err_type_t::OK;
            }
        }

        return err_type_t::COMM_ERROR;
    }
}