#include "bioscara_gripper_hardware_driver/mBaseGripper.h"
#include <fstream>
#include <iostream>
#include <cmath>

namespace bioscara_hardware_drivers
{
    BaseGripper::BaseGripper(float reduction, float offset, float min, float max, float backup_init_pos)
    {
        _reduction = reduction;
        _offset = offset;
        _min = min;
        _max = max;
        _backup_init_pos = backup_init_pos;
    }

    BaseGripper::~BaseGripper(void)
    {
        disable();
        deinit();
    }

    err_type_t BaseGripper::init(void)
    {
        return err_type_t::OK;
    }

    err_type_t BaseGripper::deinit(void)
    {
        return err_type_t::OK;
    }

    err_type_t BaseGripper::enable(void)
    {
        // try to retrieve the last recorded position
        float t;
        if (retrieve_last_position(t) != err_type_t::OK)
        {
            _pos = _backup_init_pos;
        }
        else
        {
            _pos = t;
        }
        return err_type_t::OK;
    }

    err_type_t BaseGripper::disable(void)
    {
        save_last_position(_pos);
        return err_type_t::OK;
    }

    err_type_t BaseGripper::setPosition(float width)
    {

        width = width < _min ? _min : width;
        width = width > _max ? _max : width;

        if (fabs(_pos - width) > 0.0001)
        {
            new_cmd_time = std::chrono::high_resolution_clock::now();
        }
        _pos = width;
        return err_type_t::OK;
    }

    err_type_t BaseGripper::getPosition(float &width)
    {
        /* TODO: this delay is a very bad workaround to 
        allow the gripper to move to its target. Redo with digital twin or similar*/
        auto now = std::chrono::high_resolution_clock::now();
        const std::chrono::duration<float> elapsed = now - new_cmd_time;
        if (elapsed.count() > 1.0)
        {
            _pos_get = _pos;
        }
        width = _pos_get;
        return err_type_t::OK;
    }

    err_type_t BaseGripper::setServoPosition(float /*angle*/)
    {
        return err_type_t::OK;
    }

    void BaseGripper::setReduction(float reduction)
    {
        std::cout << "Set reduction to: " << reduction << std::endl;
    }

    void BaseGripper::setOffset(float offset)
    {
        std::cout << "Set offset to: " << offset << std::endl;
    }

    err_type_t BaseGripper::save_last_position(float pos)
    {
        try
        {
            std::ofstream fout("last_pos.b", std::ios::binary);
            fout.write(reinterpret_cast<char *>(&pos), sizeof pos);
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
            std::cout << "Error writing buffer file. Last position might not be saved.\n";
            return err_type_t::ERROR;
        }
        return err_type_t::OK;
    }

    err_type_t BaseGripper::retrieve_last_position(float &pos)
    {
        float d;
        try
        {
            std::ifstream istrm("last_pos.b", std::ios::binary);
            istrm.read(reinterpret_cast<char *>(&d), sizeof d);
            if (!istrm.is_open())
            {
                std::cout << "Failed to open buffer file. Using backup initial position.\n";
                return err_type_t::ERROR;
            }
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
            std::cout << "Error reading buffer file. Using backup initial position.\n";
            return err_type_t::ERROR;
        }
        pos = d;
        return err_type_t::OK;
    }
}