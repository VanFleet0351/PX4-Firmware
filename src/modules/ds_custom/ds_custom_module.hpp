//
// Created by alpha on 2/3/20.
//

#ifndef PX4_DS_CUSTOM_MODULE_HPP
#define PX4_DS_CUSTOM_MODULE_HPP
#include <px4_platform_common/module.h>
#include <uORB/topics/ds_custom.h>
#include <errno.h>


class ds_custom_module: public ModuleBase<ds_custom_module>
{

public:
    static int print_usage(const char *reason = nullptr);
    static int custom_command(int argc, char *argv[]);

    static int task_spawn(int argc, char *argv[]);
    static ds_custom_module *instantiate(int argc, char *argv[]);

};


#endif //PX4_DS_CUSTOM_MODULE_HPP
