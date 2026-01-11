/**
 * @file uPWM.h
 * @author sbstorz and Bernd Porr, bernd.porr@glasgow.ac.uk
 * @brief Includes source code for Hardware PWM generation on Raspberry Pi 4
 * @version 0.1
 * @date 2025-05-27
 *
 * @copyright Copyright (c) 2025
 *
 */

#ifndef __RPIPWM
#define __RPIPWM

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <string>
#include <iostream>
#include <math.h>

/**
 * @brief Class to create a Pulse Width Modulated (PWM) signal on the Raspberry PI 4 and 5
 * 
 * Based on https://github.com/berndporr/rpi_pwm/blob/main/rpi_pwm.h
 * and slightly modified.
 *
 * For servo control it is important to use a hardware generated PWM, since software PWM,
 * like it is created by the lgpio library, is subject to timing jitter which can cause 
 * the servo to figet which might lead to overheating and wear.
 **/
class RPI_PWM
{
public:
    /**
     * @brief Starts the PWM
     * 
     * @param channel The GPIO channel
     * @param frequency The PWM frequency
     * @param duty_cycle The initial duty cycle of the PWM (default 0)
     * @param chip The chip number
     * @return >0 on success and -1 if an error has happened.
     **/
    int start(int channel, int frequency, float duty_cycle = 0, int chip = 2)
    {
        chippath = "/sys/class/pwm/pwmchip" + std::to_string(chip);
        pwmpath = chippath + "/pwm" + std::to_string(channel);
        std::string p = chippath + "/export";
        FILE *const fp = fopen(p.c_str(), "w");
        if (NULL == fp)
        {
            std::cerr << "PWM device does not exist. Make sure to add 'dtoverlay=pwm-2chan' to /boot/firmware/config.txt.\n";
            return -1;
        }
        const int r = fprintf(fp, "%d", channel);
        fclose(fp);
        if (r < 0)
            return r;
        usleep(100000); // it takes a while till the PWM subdir is created
        per = (int)1E9 / frequency;
        setPeriod(per);
        setDutyCycle(duty_cycle);
        enable();
        return r;
    }

    /**
     * @brief Stops the PWM
     **/
    void stop()
    {
        disable();
    }

    /**
     * @brief Destroy the RPI_PWM object
     * 
     * Invokes disable()
     * 
     */
    ~RPI_PWM()
    {
        disable();
    }

    /**
     * @brief Sets the duty cycle in percent 0 - 100.
     * @param v The duty cycle in percent.
     * @return >0 on success and -1 after an error.
     **/
    inline int setDutyCycle(float v) const
    {
        const int dc = (int)round((float)per * (v / 100.0));
        const int r = setDutyCycleNS(dc);
        return r;
    }

private:

    void setPeriod(int ns) const
    {
        writeSYS(pwmpath + "/" + "period", ns);
    }

    inline int setDutyCycleNS(int ns) const
    {
        const int r = writeSYS(pwmpath + "/" + "duty_cycle", ns);
        return r;
    }

    void enable() const
    {
        writeSYS(pwmpath + "/" + "enable", 1);
    }

    void disable() const
    {
        writeSYS(pwmpath + "/" + "enable", 0);
    }

    int per = 0;

    std::string chippath;
    std::string pwmpath;

    inline int writeSYS(std::string filename, int value) const
    {
        FILE *const fp = fopen(filename.c_str(), "w");
        if (NULL == fp)
        {
            return -1;
        }
        const int r = fprintf(fp, "%d", value);
        fclose(fp);
        return r;
    }
};

#endif