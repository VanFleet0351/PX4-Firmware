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

    PRINT_MODULE_USAGE_NAME("commander", "system");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_PARAM_FLAG('h', "Enable HIL mode", true);
    PRINT_MODULE_USAGE_COMMAND_DESCR("calibrate", "Run sensor calibration");
    PRINT_MODULE_USAGE_ARG("mag|accel|gyro|level|esc|airspeed", "Calibration type", false);
    PRINT_MODULE_USAGE_COMMAND_DESCR("check", "Run preflight checks");
    PRINT_MODULE_USAGE_COMMAND("arm");
    PRINT_MODULE_USAGE_PARAM_FLAG('f', "Force arming (do not run preflight checks)", true);
    PRINT_MODULE_USAGE_COMMAND("disarm");
    PRINT_MODULE_USAGE_COMMAND("takeoff");
    PRINT_MODULE_USAGE_COMMAND("land");
    PRINT_MODULE_USAGE_COMMAND_DESCR("transition", "VTOL transition");
    PRINT_MODULE_USAGE_COMMAND_DESCR("mode", "Change flight mode");
    PRINT_MODULE_USAGE_ARG(
            "manual|acro|offboard|stabilized|rattitude|altctl|posctl|auto:mission|auto:loiter|auto:rtl|auto:takeoff|auto:land|auto:precland",
            "Flight mode", false);
    PRINT_MODULE_USAGE_COMMAND("lockdown");
    PRINT_MODULE_USAGE_ARG("off", "Turn lockdown off", true);
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 1;
}

int ds_custom_module::custom_command(int argc, char *argv[]) {
    return print_usage("unknown command");
}

int ds_custom_module::task_spawn(int argc, char *argv[]) {
    _task_id = px4_task_spawn_cmd("ds_custom_module",
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
    ds_custom_module *instance = new ds_custom_module();

    return instance;
}