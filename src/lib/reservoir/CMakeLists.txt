#find_package (Eigen3 3.3 REQUIRED NO_MODULE)

add_library(reservoir
        reservoir_computer.cpp)

#target_link_libraries (reservoir Eigen3::Eigen)

#if(PX4_TESTING AND (${PX4_PLATFORM} MATCHES "posix"))
#    add_subdirectory(reservoir_tests)
#endif()
