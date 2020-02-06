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
    PRINT_MODULE_USAGE_COMMAND("telemetry");
    PRINT_MODULE_USAGE_ARG("pitch|roll|yaw|thrust", "Telemetry type", false);
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 1;
}

int ds_custom_module::custom_command(int argc, char *argv[]) {
    if (argc >= 1) {
        if (strcmp(argv[0], "telemetry") == 0) {
            return telemetry_read(argc - 1, argv+1);
        }
    }
    return print_usage("unknown command");

}

int ds_custom_module::telemetry_read(int argc, char *argv[]) {
    actuator_controls_s _actuators_out_0{};    //actuator controls going to the mc mixer
    uORB::Subscription _actuators_0_sub{ORB_ID(actuator_controls_0)};
    uint64_t start_time;
    _actuators_0_sub.update(&start_time, &_actuators_out_0);
    if (argc != 1) {
        return print_usage(nullptr);
    }
    if (strcmp(argv[0], "pitch") == 0) {
        PX4_INFO("%lf", (double) _actuators_out_0.control[1]);
    } else if (strcmp(argv[0], "roll") == 0) {
        PX4_INFO("%lf", (double) _actuators_out_0.control[0]);
    } else if (strcmp(argv[0], "yaw") == 0)
    {
        PX4_INFO("%lf", (double) _actuators_out_0.control[2]);
    }
    else if (strcmp(argv[0], "thrust") == 0) {
        PX4_INFO("%lf", (double) _actuators_out_0.control[3]);
    } else {
        return print_usage("unknown telemetry lookup");
    }
    return 1;
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
    while (!should_exit()) {

    }
}