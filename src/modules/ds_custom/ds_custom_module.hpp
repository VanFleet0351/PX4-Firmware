//
// Created by alpha on 2/3/20.
//

#ifndef PX4_DS_CUSTOM_MODULE_HPP
#define PX4_DS_CUSTOM_MODULE_HPP
#include <platforms/px4_module.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_controls.h>
#include <errno.h>
#include <drivers/drv_hrt.h>


class ds_custom_module: public ModuleBase<ds_custom_module>
{

public:
    static int print_usage(const char *reason = nullptr);
    static int custom_command(int argc, char *argv[]);

    static int task_spawn(int argc, char *argv[]);
    static ds_custom_module *instantiate(int argc, char *argv[]);
    static int telemetry_read(int argc, char *argv[]);
    void run() override;

private:


};


#endif //PX4_DS_CUSTOM_MODULE_HPP