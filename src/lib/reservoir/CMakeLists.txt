find_package(Eigen3 QUIET)
if(NOT EIGEN3_FOUND)
    # Fallback to cmake_modules
    find_package(Eigen QUIET)
    if(NOT EIGEN_FOUND)
        pkg_check_modules(EIGEN3 REQUIRED eigen3)
    else()
        set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
        set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})
    endif()
else()
    set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99 -Wno-deprecated-declarations")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")

include_directories(
        include
        ${Boost_INCLUDE_DIR}
        ${CMAKE_CURRENT_BINARY_DIR}
        ${EIGEN3_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}/eigen3	# Workaround for Eigen3
)

add_library(reservoir
        reservoir_computer.cpp)


#if(PX4_TESTING AND (${PX4_PLATFORM} MATCHES "posix"))
#    add_subdirectory(reservoir_tests)
#endif()
