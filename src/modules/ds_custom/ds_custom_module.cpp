//
// Created by alpha on 2/3/20.
//

#include "ds_custom_module.hpp"

extern "C" __EXPORT int ds_custom_main(int argc, char *argv[]) {
    ds_custom_module::main(argc, argv);
    return 0;
}

int ds_custom_module::print_usage(const char *reason) {
    if (reason) {
        PX4_INFO("%s", reason);
    }

    PRINT_MODULE_DESCRIPTION(
            R"DESCR_STR(
### Description
The commander module contains the state machine for mode switching and failsafe behavior.
)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("ds_custom", "system");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_COMMAND("message");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 1;
}

int ds_custom_module::custom_command(int argc, char *argv[]) {
    return print_usage("unknown command");
}

int ds_custom_module::task_spawn(int argc, char *argv[]) {
    _task_id = px4_task_spawn_cmd("ds_custom",
                                  SCHED_DEFAULT,
                                  SCHED_PRIORITY_DEFAULT + 40,
                                  3250,
                                  (px4_main_t) &run_trampoline,
                                  (char *const *) argv);

    if (_task_id < 0) {
        _task_id = -1;
        return -errno;
    }

    return 0;
}

ds_custom_module *ds_custom_module::instantiate(int argc, char *argv[]) {
    auto *instance = new ds_custom_module();

    return instance;
}

void ds_custom_module::run() {
    //hrt_abstime start_time = hrt_absolute_time();
    while (!should_exit()) {

    }
}