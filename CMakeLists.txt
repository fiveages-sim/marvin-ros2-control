cmake_minimum_required(VERSION 3.10)
project(marvin_ros2_control)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# Generate symbols for IDE indexer
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(gripper_hardware_common REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(EIGEN3 REQUIRED eigen3)

set(HEADER_KINE_DIR "${CMAKE_SOURCE_DIR}/external/TJ_FX_ROBOT_CONTRL_SDK/kinematicsSDK/")
set(HEADER_CONTROL_DIR "${CMAKE_SOURCE_DIR}/external/TJ_FX_ROBOT_CONTRL_SDK/contrlSDK/")
set(LIB_KINE_DIR "${CMAKE_SOURCE_DIR}/external/TJ_FX_ROBOT_CONTRL_SDK/kinematicsSDK/")
set(LIB_CONTROL_DIR "${CMAKE_SOURCE_DIR}/external/TJ_FX_ROBOT_CONTRL_SDK/contrlSDK/")

# Build kinematics SDK first (libKine.so is produced by make; not present at CMake configure time)
add_custom_command(
    OUTPUT ${LIB_KINE_DIR}/libKine.so
    COMMAND make -f makefile libKine.so
    WORKING_DIRECTORY ${LIB_KINE_DIR}
    COMMENT "Building libKine.so (kinematics SDK)"
)
add_custom_target(build_kine_sdk ALL DEPENDS ${LIB_KINE_DIR}/libKine.so)

# build controlSDK
add_custom_command(
    OUTPUT ${LIB_CONTROL_DIR}/libMarvinSDK.so
    COMMAND make -f makefile libMarvinSDK.so
    WORKING_DIRECTORY ${LIB_CONTROL_DIR}
    COMMENT "Building Marvin SDK with Makefile"
)
add_custom_target(build_marvin_sdk ALL DEPENDS ${LIB_CONTROL_DIR}/libMarvinSDK.so)

# Full path: find_library at configure time fails because libKine.so does not exist until build_kine_sdk runs
set(EXTERNAL_KINEMATICSO "${LIB_KINE_DIR}/libKine.so")


# Create library
add_library(${PROJECT_NAME} SHARED
        src/marvin_hardware.cpp
        src/tool/modbus_io.cpp
        src/tool/grippers/jodell/jd_gripper.cpp
)

add_dependencies(${PROJECT_NAME} build_kine_sdk build_marvin_sdk)

set_target_properties(${PROJECT_NAME} PROPERTIES
    BUILD_RPATH "${LIB_KINE_DIR};${LIB_CONTROL_DIR}"
    INSTALL_RPATH "$ORIGIN"
    INSTALL_RPATH_USE_LINK_PATH TRUE
)

target_include_directories(${PROJECT_NAME} PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        # "$<BUILD_INTERFACE:${HEADER_KINE_DIR}>"
        # "$<BUILD_INTERFACE:${HEADER_CONTROL_DIR}>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
        PRIVATE src
        PRIVATE ${HEADER_KINE_DIR}
        PRIVATE ${HEADER_CONTROL_DIR}
        PRIVATE ${EIGEN3_INCLUDE_DIRS}
)

# Add library directories for linking
target_link_directories(${PROJECT_NAME} PRIVATE
    ${LIB_KINE_DIR}
    ${LIB_CONTROL_DIR}
)

ament_target_dependencies(${PROJECT_NAME}
        hardware_interface
        rclcpp
        rclcpp_lifecycle
        pluginlib
        gripper_hardware_common
)

# # Detect architecture and set library path
# if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
#     set(XCORE_LIB_DIR "aarch64")
# else()
#     set(XCORE_LIB_DIR "x86_64")
# endif()

# # Link xCoreSDK libraries
target_link_libraries(${PROJECT_NAME}
        ${EXTERNAL_KINEMATICSO}
        MarvinSDK
        pthread
        rt
)

pluginlib_export_plugin_description_file(hardware_interface marvin_ros2_control.xml)

# Minimal example: connect to robot and initialize dual Changingtek 90D gripper (always build if source exists)
set(MARVIN_EXAMPLE_SRC "${CMAKE_CURRENT_SOURCE_DIR}/examples/marvin_gripper_init_example.cpp")
if(EXISTS "${MARVIN_EXAMPLE_SRC}")
    add_executable(marvin_gripper_init_example "${MARVIN_EXAMPLE_SRC}")
    add_dependencies(marvin_gripper_init_example build_kine_sdk build_marvin_sdk)
    target_include_directories(marvin_gripper_init_example PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${HEADER_KINE_DIR}
        ${HEADER_CONTROL_DIR}
        ${EIGEN3_INCLUDE_DIRS}
    )
    target_link_directories(marvin_gripper_init_example PRIVATE
        ${LIB_KINE_DIR}
        ${LIB_CONTROL_DIR}
    )
    set_target_properties(marvin_gripper_init_example PROPERTIES
        BUILD_RPATH "${LIB_KINE_DIR};${LIB_CONTROL_DIR}"
        INSTALL_RPATH "$ORIGIN;$ORIGIN/../lib"
        INSTALL_RPATH_USE_LINK_PATH TRUE
    )
    ament_target_dependencies(marvin_gripper_init_example rclcpp gripper_hardware_common)
    target_link_libraries(marvin_gripper_init_example
        ${PROJECT_NAME}
        ${EXTERNAL_KINEMATICSO}
        MarvinSDK
        pthread
        rt
    )
    install(TARGETS marvin_gripper_init_example
        RUNTIME DESTINATION lib/${PROJECT_NAME}
    )
    message(STATUS "marvin_ros2_control: building and installing marvin_gripper_init_example")
endif()

# Hand init example: connect to robot and initialize dual LinkerHand (dexterous hand)
set(MARVIN_HAND_EXAMPLE_SRC "${CMAKE_CURRENT_SOURCE_DIR}/examples/marvin_hand_init_example.cpp")
if(EXISTS "${MARVIN_HAND_EXAMPLE_SRC}")
    add_executable(marvin_hand_init_example "${MARVIN_HAND_EXAMPLE_SRC}")
    add_dependencies(marvin_hand_init_example build_kine_sdk build_marvin_sdk)
    target_include_directories(marvin_hand_init_example PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${HEADER_KINE_DIR}
        ${HEADER_CONTROL_DIR}
        ${EIGEN3_INCLUDE_DIRS}
    )
    target_link_directories(marvin_hand_init_example PRIVATE
        ${LIB_KINE_DIR}
        ${LIB_CONTROL_DIR}
    )
    set_target_properties(marvin_hand_init_example PROPERTIES
        BUILD_RPATH "${LIB_KINE_DIR};${LIB_CONTROL_DIR}"
        INSTALL_RPATH "$ORIGIN;$ORIGIN/../lib"
        INSTALL_RPATH_USE_LINK_PATH TRUE
    )
    ament_target_dependencies(marvin_hand_init_example rclcpp gripper_hardware_common)
    target_link_libraries(marvin_hand_init_example
        ${PROJECT_NAME}
        ${EXTERNAL_KINEMATICSO}
        MarvinSDK
        pthread
        rt
    )
    install(TARGETS marvin_hand_init_example
        RUNTIME DESTINATION lib/${PROJECT_NAME}
    )
    message(STATUS "marvin_ros2_control: building and installing marvin_hand_init_example")
endif()

# Standalone async raw 485 test: SDK only, no ROS, no marvin_ros2_control library
set(ASYNC_RAW_SDK_ONLY_SRC "${CMAKE_CURRENT_SOURCE_DIR}/examples/async_raw_485_test_sdk_only.cpp")
if(EXISTS "${ASYNC_RAW_SDK_ONLY_SRC}")
    add_executable(async_raw_485_test_sdk_only "${ASYNC_RAW_SDK_ONLY_SRC}")
    add_dependencies(async_raw_485_test_sdk_only build_marvin_sdk)
    target_include_directories(async_raw_485_test_sdk_only PRIVATE ${HEADER_CONTROL_DIR})
    target_link_directories(async_raw_485_test_sdk_only PRIVATE ${LIB_CONTROL_DIR})
    set_target_properties(async_raw_485_test_sdk_only PROPERTIES
        BUILD_RPATH "${LIB_KINE_DIR};${LIB_CONTROL_DIR}"
        INSTALL_RPATH "$ORIGIN;$ORIGIN/../lib"
        INSTALL_RPATH_USE_LINK_PATH TRUE
    )
    target_link_libraries(async_raw_485_test_sdk_only MarvinSDK pthread)
    install(TARGETS async_raw_485_test_sdk_only
        RUNTIME DESTINATION lib/${PROJECT_NAME}
    )
    message(STATUS "marvin_ros2_control: building and installing async_raw_485_test_sdk_only (SDK only)")
endif()

install(
        DIRECTORY include/ 
        DESTINATION include/${PROJECT_NAME}
)

install(FILES
    ${LIB_KINE_DIR}/libKine.so
    ${LIB_CONTROL_DIR}/libMarvinSDK.so
    DESTINATION lib
    OPTIONAL
)

install(
        TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif ()

ament_export_dependencies(${dependencies})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
