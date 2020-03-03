find_package(Eigen3 REQUIRED)
set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

include_directories(
        include
        ${EIGEN3_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}/eigen3	# Workaround for Eigen3
)

add_library(reservoir
        reservoir_computer.cpp)


#if(PX4_TESTING AND (${PX4_PLATFORM} MATCHES "posix"))
#    add_subdirectory(reservoir_tests)
#endif()
