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

# Build kinematics SDK (libKine.so) if missing, and auto-rebuild if format is invalid.
set(KINE_SO_PATH "${LIB_KINE_DIR}/libKine.so")

# If libKine.so is missing, build it from sources with g++ (Linux .so).
if(NOT EXISTS "${KINE_SO_PATH}")
    add_custom_command(
        OUTPUT "${KINE_SO_PATH}"
        COMMAND /bin/bash -lc "g++ *.cpp -Wall -w -O2 -fPIC -shared -o libKine.so -lpthread -lrt"
        WORKING_DIRECTORY "${LIB_KINE_DIR}"
        COMMENT "Building libKine.so (kinematics SDK, Linux g++)"
        VERBATIM
    )
endif()
add_custom_target(build_kine_sdk ALL DEPENDS "${KINE_SO_PATH}")

# If libKine.so exists but is not a Linux ELF (e.g. a Windows DLL renamed to .so),
# try to rebuild it from source in kinematicsSDK/.
option(MARVIN_AUTO_REBUILD_KINE "Auto rebuild kinematicsSDK libKine.so if format is invalid" ON)
if(MARVIN_AUTO_REBUILD_KINE AND EXISTS "${KINE_SO_PATH}")
    find_program(FILE_CMD file)
    if(FILE_CMD)
        execute_process(
            COMMAND "${FILE_CMD}" -b "${KINE_SO_PATH}"
            OUTPUT_VARIABLE KINE_SO_FILEINFO
            OUTPUT_STRIP_TRAILING_WHITESPACE
            ERROR_QUIET
        )
        if(KINE_SO_FILEINFO MATCHES "PE32" OR KINE_SO_FILEINFO MATCHES "MS Windows")
            message(WARNING
                "marvin_ros2_control: ${KINE_SO_PATH} is not a Linux ELF shared library (${KINE_SO_FILEINFO}). "
                "Attempting to rebuild via direct g++ in ${LIB_KINE_DIR} ..."
            )
            execute_process(
                COMMAND "${CMAKE_COMMAND}" -E rm -f "${KINE_SO_PATH}"
                WORKING_DIRECTORY "${LIB_KINE_DIR}"
                OUTPUT_QUIET
                ERROR_QUIET
            )
            execute_process(
                COMMAND /bin/bash -lc "g++ *.cpp -Wall -w -O2 -fPIC -shared -o libKine.so -lpthread -lrt"
                WORKING_DIRECTORY "${LIB_KINE_DIR}"
                RESULT_VARIABLE KINE_BUILD_RC
                OUTPUT_VARIABLE KINE_BUILD_OUT
                ERROR_VARIABLE KINE_BUILD_ERR
            )
            if(NOT KINE_BUILD_RC EQUAL 0)
                message(FATAL_ERROR
                    "marvin_ros2_control: failed to rebuild libKine.so via g++ (rc=${KINE_BUILD_RC}).\n"
                    "stdout:\n${KINE_BUILD_OUT}\n"
                    "stderr:\n${KINE_BUILD_ERR}\n"
                    "Please obtain/build a Linux x86_64 ELF libKine.so."
                )
            endif()
            execute_process(
                COMMAND "${FILE_CMD}" -b "${KINE_SO_PATH}"
                OUTPUT_VARIABLE KINE_SO_FILEINFO2
                OUTPUT_STRIP_TRAILING_WHITESPACE
                ERROR_QUIET
            )
            if(KINE_SO_FILEINFO2 MATCHES "PE32" OR KINE_SO_FILEINFO2 MATCHES "MS Windows")
                message(FATAL_ERROR
                    "marvin_ros2_control: after rebuild, ${KINE_SO_PATH} is still not a Linux ELF shared library:\n"
                    "  ${KINE_SO_FILEINFO2}\n"
                    "Please obtain/build the correct Linux version of libKine.so."
                )
            else()
                message(STATUS "marvin_ros2_control: libKine.so rebuilt OK (${KINE_SO_FILEINFO2})")
            endif()
        endif()
    else()
        message(WARNING "marvin_ros2_control: 'file' command not found; cannot validate/rebuild libKine.so format.")
    endif()
endif()

# build controlSDK
add_custom_command(
    OUTPUT ${LIB_CONTROL_DIR}/libMarvinSDK.so
    COMMAND make -f makefile libMarvinSDK.so
    WORKING_DIRECTORY ${LIB_CONTROL_DIR}
    COMMENT "Building Marvin SDK with Makefile"
)
add_custom_target(build_marvin_sdk ALL DEPENDS ${LIB_CONTROL_DIR}/libMarvinSDK.so)

set(EXTERNAL_KINEMATICSO "${KINE_SO_PATH}")
#
# NOTE:
# Do not hard-fail at *configure time* if libKine.so is missing.
# We deliberately allow building from sources via the build_kine_sdk target,
# so repos can avoid tracking prebuilt .so binaries in git.

## Note: libMarvinSDK.so is built during compilation, so we link it directly via target_link_directories
## instead of using find_library which runs at configure time


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

# Install Python tools as ROS executables (so `ros2 run marvin_ros2_control <tool>` works).
install(PROGRAMS
    scripts/tool_dyn_identify_wizard.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME tool_dyn_identify_wizard
)

# Install SDK Python bindings and identification template data (full upstream SDK only).
# Trimmed mirror repos (contrlSDK + kinematicsSDK only) omit SDK_PYTHON/DEMO_PYTHON; skip install in that case.
set(_TJ_SDK_ROOT "${CMAKE_SOURCE_DIR}/external/TJ_FX_ROBOT_CONTRL_SDK")
if(EXISTS "${_TJ_SDK_ROOT}/SDK_PYTHON")
    install(
        DIRECTORY external/TJ_FX_ROBOT_CONTRL_SDK/SDK_PYTHON
        DESTINATION share/${PROJECT_NAME}/TJ_FX_ROBOT_CONTRL_SDK
    )
    # Keep SDK_PYTHON unmodified (upstream behavior loads .so next to the .py files),
    # but do NOT track binaries in git: we install the freshly built .so here.
    install(FILES
        ${LIB_KINE_DIR}/libKine.so
        ${LIB_CONTROL_DIR}/libMarvinSDK.so
        DESTINATION share/${PROJECT_NAME}/TJ_FX_ROBOT_CONTRL_SDK/SDK_PYTHON
        OPTIONAL
    )
endif()
if(EXISTS "${_TJ_SDK_ROOT}/DEMO_PYTHON/LoadData_ccs_right/LoadData/CfgFile")
    install(
        DIRECTORY external/TJ_FX_ROBOT_CONTRL_SDK/DEMO_PYTHON/LoadData_ccs_right/LoadData/CfgFile
                  external/TJ_FX_ROBOT_CONTRL_SDK/DEMO_PYTHON/LoadData_ccs_right/LoadData/IdenTraj
        DESTINATION share/${PROJECT_NAME}/TJ_FX_ROBOT_CONTRL_SDK/DEMO_PYTHON/LoadData_ccs_right/LoadData
    )
endif()

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
