cmake_minimum_required(VERSION 3.10)
project(legged_controllers)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

## libtorch path
## TODO: change absolute path to param
set(Torch_DIR /home/luohx/Library/libtorch/share/cmake/Torch)

## Find catkin macros and libraries
find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        legged_common
        legged_interface
        legged_wbc
        legged_estimation
        controller_interface
        ocs2_legged_robot_ros
        ocs2_self_collision_visualization
        angles
        )

find_package(Torch REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(pinocchio REQUIRED pinocchio)

catkin_package(
        INCLUDE_DIRS
        include
        LIBRARIES
        ${PROJECT_NAME}
        CATKIN_DEPENDS
        roscpp
        legged_common
        legged_interface
        legged_wbc
        legged_estimation
        controller_interface
        ocs2_legged_robot_ros
        ocs2_self_collision_visualization
        angles
)

###########
## Build ##
###########

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${pinocchio_INCLUDE_DIRS}
        ${TORCH_INCLUDE_DIRS}
)

link_directories(
        ${pinocchio_LIBRARY_DIRS}
)

## Declare a cpp library
add_library(${PROJECT_NAME}
        src/LeggedController.cpp
        )

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${pinocchio_LIBRARIES}
        )

target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

add_library(rl_${PROJECT_NAME}
        src/legged_rl_controller.cpp
        )

target_link_libraries(rl_${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${pinocchio_LIBRARIES}
        ${TORCH_LIBRARIES}
        )
set_property(TARGET rl_${PROJECT_NAME} PROPERTY CXX_STANDARD 14)

# RL policy node
add_executable(rl_policy_node
        src/rl_policy_node.cpp
        )
add_dependencies(rl_policy_node
        ${PROJECT_NAME}
        ${catkin_EXPORTED_TARGETS}
        )
target_link_libraries(rl_policy_node
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${TORCH_LIBRARIES}
        )
set_property(TARGET rl_policy_node PROPERTY CXX_STANDARD 14)

# Target trajectories publish node
add_executable(legged_target_trajectories_publisher
        src/TargetTrajectoriesPublisher.cpp
        )
add_dependencies(legged_target_trajectories_publisher
        ${PROJECT_NAME}
        ${catkin_EXPORTED_TARGETS}
        )
target_link_libraries(legged_target_trajectories_publisher
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        )
target_compile_options(legged_target_trajectories_publisher PRIVATE ${OCS2_CXX_FLAGS})

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
    message(STATUS "Run clang tooling for target " ${PROJECT_NAME})
    add_clang_tooling(
            TARGETS ${PROJECT_NAME} legged_target_trajectories_publisher
            SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
            CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
            CF_WERROR
    )
endif (cmake_clang_tools_FOUND)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(TARGETS rl_${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        )

# Mark other files for installation
install(
        FILES legged_rl_controllers_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)