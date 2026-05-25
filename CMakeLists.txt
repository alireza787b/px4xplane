#==============================================================================
# px4xplane - Cross-Platform CMake Build System
# X-Plane Plugin for PX4 SITL Integration
# Repository: alireza787b/px4xplane
#
# Supports: Windows (MSVC/MinGW), Linux (GCC/Clang), macOS (Clang)
# Build: mkdir build && cd build && cmake .. && cmake --build .
#==============================================================================

cmake_minimum_required(VERSION 3.15)
project(px4xplane VERSION 3.4.41 LANGUAGES CXX)

# C++ Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Build type default to Release
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

#==============================================================================
# Platform Detection
#==============================================================================

if(WIN32)
    set(PLUGIN_PLATFORM "win")
    set(PLUGIN_EXT ".xpl")
    set(LIB_SUFFIX ".lib")
    set(SDK_LIB_DIR "${CMAKE_SOURCE_DIR}/lib/SDK/Libraries/Win")
elseif(APPLE)
    set(PLUGIN_PLATFORM "mac")
    set(PLUGIN_EXT ".xpl")
    set(LIB_SUFFIX ".framework")
    set(SDK_LIB_DIR "${CMAKE_SOURCE_DIR}/lib/SDK/Libraries/Mac")
elseif(UNIX)
    set(PLUGIN_PLATFORM "lin")
    set(PLUGIN_EXT ".xpl")
    set(LIB_SUFFIX ".so")
    set(SDK_LIB_DIR "${CMAKE_SOURCE_DIR}/lib/SDK/Libraries/Lin")
else()
    message(FATAL_ERROR "Unsupported platform")
endif()

message(STATUS "Building for platform: ${PLUGIN_PLATFORM}")
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

#==============================================================================
# X-Plane Plugin Configuration
#==============================================================================

# Output directory structure: build/{platform}/{config}/px4xplane/64/
set(PACKAGE_ROOT "${CMAKE_BINARY_DIR}/${PLUGIN_PLATFORM}/${CMAKE_BUILD_TYPE}")
set(OUTPUT_DIR "${CMAKE_BINARY_DIR}/${PLUGIN_PLATFORM}/${CMAKE_BUILD_TYPE}/px4xplane/64")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${OUTPUT_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${OUTPUT_DIR})

#==============================================================================
# Source Files
#==============================================================================

set(SOURCES
    src/AccelCalibration.cpp
    src/ActuatorSafety.cpp
    src/ConfigManager.cpp
    src/configReader.cpp
    src/ConnectionManager.cpp
    src/ConnectionStatusHUD.cpp
    src/DataRefManager.cpp
    src/MAVLinkManager.cpp
    src/px4xplane.cpp
    src/TimeManager.cpp
    src/TimestampProvider.cpp
    src/UIConstants.cpp
    src/UIHandler.cpp
)

option(PX4XPLANE_BUILD_TESTS "Build offline px4xplane unit tests" OFF)

set(HEADERS
    include/AccelCalibration.h
    include/ActuatorSafety.h
    include/ConfigManager.h
    include/configReader.h
    include/ConnectionManager.h
    include/ConnectionStatusHUD.h
    include/DataRefManager.h
    include/DataRefProvider.h
    include/MAVLinkManager.h
    include/TimeManager.h
    include/TimestampProvider.h
    include/UIConstants.h
    include/UIHandler.h
    include/VersionInfo.h
    config/ini.h
    lib/XYZgeomag/src/XYZgeomag.hpp
)

#==============================================================================
# Create Plugin Library
#==============================================================================

add_library(${PROJECT_NAME} MODULE ${SOURCES} ${HEADERS})

# Set output name to match X-Plane convention: win.xpl, lin.xpl, mac.xpl
set_target_properties(${PROJECT_NAME} PROPERTIES
    PREFIX ""
    OUTPUT_NAME "${PLUGIN_PLATFORM}"
    SUFFIX "${PLUGIN_EXT}"
)

# Fix Windows MSVC multi-config generator (creates extra Release/Debug subdirectory)
# Set per-configuration output directories to avoid nested config folders
if(WIN32 AND MSVC)
    foreach(CONFIG Release Debug MinSizeRel RelWithDebInfo)
        string(TOUPPER ${CONFIG} CONFIG_UPPER)
        set_target_properties(${PROJECT_NAME} PROPERTIES
            RUNTIME_OUTPUT_DIRECTORY_${CONFIG_UPPER} ${OUTPUT_DIR}
            LIBRARY_OUTPUT_DIRECTORY_${CONFIG_UPPER} ${OUTPUT_DIR}
        )
    endforeach()
endif()

#==============================================================================
# Include Directories
#==============================================================================

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/lib/SDK/CHeaders/XPLM
    ${CMAKE_SOURCE_DIR}/lib/SDK/CHeaders/Widgets
    ${CMAKE_SOURCE_DIR}/lib/mavlink/c_library_v2
    ${CMAKE_SOURCE_DIR}/lib/mavlink/c_library_v2/common
    ${CMAKE_SOURCE_DIR}/lib/mavlink/c_library_v2/all
    ${CMAKE_SOURCE_DIR}/lib/simpleini
    ${CMAKE_SOURCE_DIR}/lib/Eigen
    ${CMAKE_SOURCE_DIR}/lib/XYZgeomag/src
    ${CMAKE_SOURCE_DIR}/config
)

#==============================================================================
# Platform-Specific Compiler Flags
#==============================================================================

if(WIN32)
    # Windows (MSVC)
    target_compile_definitions(${PROJECT_NAME} PRIVATE
        WIN32
        _WINDOWS
        _USRDLL
        IBM=1
        XPLM200=1
        XPLM210=1
        XPLM300=1
        XPLM301=1
        XPLM302=1
        XPLM303=1
        XPLM400=1
        _USE_MATH_DEFINES
        _CRT_SECURE_NO_WARNINGS
        _CRT_NONSTDC_NO_DEPRECATE
        NOMINMAX
        WINVER=0x0A00
        _WIN32_WINNT=0x0A00
        $<$<CONFIG:Debug>:_DEBUG>
        $<$<CONFIG:Release>:NDEBUG>
    )

    # MSVC-specific flags
    if(MSVC)
        target_compile_options(${PROJECT_NAME} PRIVATE
            /W3
            /MP  # Multi-processor compilation
            $<$<CONFIG:Release>:/O2 /GL /Oi>  # Optimization
        )
        target_link_options(${PROJECT_NAME} PRIVATE
            $<$<CONFIG:Release>:/LTCG>  # Link-time code generation
        )
    elseif(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
        target_link_options(${PROJECT_NAME} PRIVATE
            -static-libgcc
            -static-libstdc++
        )
    endif()

elseif(APPLE)
    # macOS (Clang)
    target_compile_definitions(${PROJECT_NAME} PRIVATE
        APL=1
        XPLM200=1
        XPLM210=1
        XPLM300=1
        XPLM301=1
        XPLM302=1
        XPLM303=1
        XPLM400=1
    )

    target_compile_options(${PROJECT_NAME} PRIVATE
        -Wall
        -Wextra
        -fvisibility=hidden
        -fPIC
        $<$<CONFIG:Release>:-O3>
        $<$<CONFIG:Debug>:-g -O0>
    )

    # macOS-specific linker flags
    # X-Plane plugins use flat namespace with dynamic lookup (symbols resolved at runtime by X-Plane)
    set_target_properties(${PROJECT_NAME} PROPERTIES
        LINK_FLAGS "-flat_namespace -undefined dynamic_lookup"
    )

elseif(UNIX)
    # Linux (GCC/Clang)
    target_compile_definitions(${PROJECT_NAME} PRIVATE
        LIN=1
        XPLM200=1
        XPLM210=1
        XPLM300=1
        XPLM301=1
        XPLM302=1
        XPLM303=1
        XPLM400=1
    )

    target_compile_options(${PROJECT_NAME} PRIVATE
        -Wall
        -Wextra
        -fvisibility=hidden
        -fPIC
        $<$<CONFIG:Release>:-O3>
        $<$<CONFIG:Debug>:-g -O0>
    )

    # Linux-specific linker flags
    # Note: -Wl,--allow-shlib-undefined allows undefined symbols (resolved by X-Plane at runtime)
    set_target_properties(${PROJECT_NAME} PROPERTIES
        LINK_FLAGS "-rdynamic -nodefaultlibs -Wl,--allow-shlib-undefined"
    )
endif()

#==============================================================================
# Link Libraries
#==============================================================================

if(WIN32)
    # Windows libraries
    target_link_libraries(${PROJECT_NAME} PRIVATE
        ${SDK_LIB_DIR}/XPLM_64.lib
        ${SDK_LIB_DIR}/XPWidgets_64.lib
        opengl32
        ws2_32
    )

elseif(APPLE)
    # macOS frameworks
    # Note: XPLM and XPWidgets are NOT linked at compile time for X-Plane plugins
    # They use flat namespace with undefined dynamic_lookup (resolved at runtime by X-Plane)
    target_link_libraries(${PROJECT_NAME} PRIVATE
        "-framework OpenGL"
    )

elseif(UNIX)
    # Linux shared libraries
    target_link_libraries(${PROJECT_NAME} PRIVATE
        ${SDK_LIB_DIR}/XPLM_64.so
        ${SDK_LIB_DIR}/XPWidgets_64.so
        GL
    )
endif()

#==============================================================================
# Post-Build: Copy Configuration Files
#==============================================================================

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${CMAKE_SOURCE_DIR}/config/config.ini
        ${OUTPUT_DIR}/config.ini
    COMMENT "Copying config.ini to 64/ directory (with plugin binary)"
)

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${CMAKE_SOURCE_DIR}/config/config_schema.json
        ${OUTPUT_DIR}/config_schema.json
    COMMENT "Copying config_schema.json to 64/ directory"
)

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${CMAKE_SOURCE_DIR}/README.md
        ${OUTPUT_DIR}/../README.md
    COMMENT "Copying README.md to plugin root directory"
)

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${CMAKE_SOURCE_DIR}/docs/ALIA_XPLANE12_TEST.md
        ${OUTPUT_DIR}/../ALIA_XPLANE12_TEST.md
    COMMENT "Copying Alia X-Plane test card to plugin root directory"
)

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${CMAKE_SOURCE_DIR}/docs/EHANG184_XPLANE12_TEST.md
        ${OUTPUT_DIR}/../EHANG184_XPLANE12_TEST.md
    COMMENT "Copying Ehang 184 X-Plane test card to plugin root directory"
)

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${CMAKE_SOURCE_DIR}/docs/QUADTAILSITTER_XPLANE12_TEST.md
        ${OUTPUT_DIR}/../QUADTAILSITTER_XPLANE12_TEST.md
    COMMENT "Copying QuadTailsitter X-Plane test card to plugin root directory"
)

if(EXISTS "${CMAKE_SOURCE_DIR}/aircraft/QuadTailsitter")
    add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E make_directory
            ${PACKAGE_ROOT}/X-Plane_Aircraft
        COMMAND ${CMAKE_COMMAND} -E copy_directory
            ${CMAKE_SOURCE_DIR}/aircraft/QuadTailsitter
            ${PACKAGE_ROOT}/X-Plane_Aircraft/QuadTailsitter
        COMMENT "Copying QuadTailsitter aircraft source to package output"
    )
endif()

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory
        ${OUTPUT_DIR}/../docs/assets
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${CMAKE_SOURCE_DIR}/docs/config-editor.html
        ${OUTPUT_DIR}/../docs/config-editor.html
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${CMAKE_SOURCE_DIR}/docs/assets/config-editor.js
        ${OUTPUT_DIR}/../docs/assets/config-editor.js
    COMMENT "Copying config editor docs to plugin package"
)

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory
        ${OUTPUT_DIR}/../tools
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${CMAKE_SOURCE_DIR}/tools/check_px4_airframe_params.py
        ${OUTPUT_DIR}/../tools/check_px4_airframe_params.py
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${CMAKE_SOURCE_DIR}/tools/analyze_qtailsitter_design.py
        ${OUTPUT_DIR}/../tools/analyze_qtailsitter_design.py
    COMMENT "Copying validation tools to plugin package"
)

# Create px4_airframes directory for organized PX4 parameter files
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory
        ${OUTPUT_DIR}/../px4_airframes
    COMMENT "Creating px4_airframes directory"
)

# Copy PX4 parameter files to px4_airframes subdirectory
file(GLOB PX4_PARAMS "${CMAKE_SOURCE_DIR}/config/px4_params/*")
foreach(PARAM_FILE ${PX4_PARAMS})
    get_filename_component(PARAM_NAME ${PARAM_FILE} NAME)
    add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${PARAM_FILE}
            ${OUTPUT_DIR}/../px4_airframes/${PARAM_NAME}
        COMMENT "Copying PX4 airframe parameter: ${PARAM_NAME}"
    )
endforeach()

# Refresh package assets even when the plugin binary itself does not relink.
# Without this target, config/docs/airframe-only edits can leave an existing
# build package carrying stale files.
set(PACKAGE_ASSET_COMMANDS
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${CMAKE_SOURCE_DIR}/config/config.ini
        ${OUTPUT_DIR}/config.ini
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${CMAKE_SOURCE_DIR}/config/config_schema.json
        ${OUTPUT_DIR}/config_schema.json
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${CMAKE_SOURCE_DIR}/README.md
        ${OUTPUT_DIR}/../README.md
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${CMAKE_SOURCE_DIR}/docs/ALIA_XPLANE12_TEST.md
        ${OUTPUT_DIR}/../ALIA_XPLANE12_TEST.md
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${CMAKE_SOURCE_DIR}/docs/EHANG184_XPLANE12_TEST.md
        ${OUTPUT_DIR}/../EHANG184_XPLANE12_TEST.md
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${CMAKE_SOURCE_DIR}/docs/QUADTAILSITTER_XPLANE12_TEST.md
        ${OUTPUT_DIR}/../QUADTAILSITTER_XPLANE12_TEST.md
    COMMAND ${CMAKE_COMMAND} -E make_directory
        ${OUTPUT_DIR}/../docs/assets
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${CMAKE_SOURCE_DIR}/docs/config-editor.html
        ${OUTPUT_DIR}/../docs/config-editor.html
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${CMAKE_SOURCE_DIR}/docs/assets/config-editor.js
        ${OUTPUT_DIR}/../docs/assets/config-editor.js
    COMMAND ${CMAKE_COMMAND} -E make_directory
        ${OUTPUT_DIR}/../tools
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${CMAKE_SOURCE_DIR}/tools/check_px4_airframe_params.py
        ${OUTPUT_DIR}/../tools/check_px4_airframe_params.py
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${CMAKE_SOURCE_DIR}/tools/analyze_qtailsitter_design.py
        ${OUTPUT_DIR}/../tools/analyze_qtailsitter_design.py
    COMMAND ${CMAKE_COMMAND} -E make_directory
        ${OUTPUT_DIR}/../px4_airframes
)

foreach(PARAM_FILE ${PX4_PARAMS})
    get_filename_component(PARAM_NAME ${PARAM_FILE} NAME)
    list(APPEND PACKAGE_ASSET_COMMANDS
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${PARAM_FILE}
            ${OUTPUT_DIR}/../px4_airframes/${PARAM_NAME}
    )
endforeach()

add_custom_target(package_assets ALL
    ${PACKAGE_ASSET_COMMANDS}
    COMMENT "Refreshing packaged config, docs, and PX4 airframe files"
    VERBATIM
)
add_dependencies(package_assets ${PROJECT_NAME})

#==============================================================================
# Installation (Optional)
#==============================================================================

# Example: Install to X-Plane plugin directory
# Uncomment and modify path as needed
# set(XPLANE_PLUGIN_DIR "$ENV{HOME}/X-Plane 12/Resources/plugins")
# install(TARGETS ${PROJECT_NAME}
#     LIBRARY DESTINATION ${XPLANE_PLUGIN_DIR}/px4xplane/64
# )

#==============================================================================
# Build Summary
#==============================================================================

message(STATUS "==========================================================")

if(PX4XPLANE_BUILD_TESTS)
    enable_testing()

    add_executable(actuator_safety_test
        tests/actuator_safety_test.cpp
        src/ActuatorSafety.cpp
    )
    target_include_directories(actuator_safety_test PRIVATE
        ${CMAKE_SOURCE_DIR}/include
    )
    add_test(NAME actuator_safety_test COMMAND actuator_safety_test)

    add_executable(timestamp_provider_test
        tests/timestamp_provider_test.cpp
        src/TimestampProvider.cpp
    )
    target_include_directories(timestamp_provider_test PRIVATE
        ${CMAKE_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}/lib/SDK/CHeaders/XPLM
        ${CMAKE_SOURCE_DIR}/lib/SDK/CHeaders/Widgets
        ${CMAKE_SOURCE_DIR}/lib/simpleini
        ${CMAKE_SOURCE_DIR}/lib/Eigen
        ${CMAKE_SOURCE_DIR}/config
    )
    if(WIN32)
        target_compile_definitions(timestamp_provider_test PRIVATE
            IBM=1 XPLM200=1 XPLM210=1 XPLM300=1 XPLM301=1 XPLM302=1 XPLM303=1 XPLM400=1
        )
    elseif(APPLE)
        target_compile_definitions(timestamp_provider_test PRIVATE
            APL=1 XPLM200=1 XPLM210=1 XPLM300=1 XPLM301=1 XPLM302=1 XPLM303=1 XPLM400=1
        )
    elseif(UNIX)
        target_compile_definitions(timestamp_provider_test PRIVATE
            LIN=1 XPLM200=1 XPLM210=1 XPLM300=1 XPLM301=1 XPLM302=1 XPLM303=1 XPLM400=1
        )
    endif()
    add_test(NAME timestamp_provider_test COMMAND timestamp_provider_test)

    find_package(Python3 COMPONENTS Interpreter QUIET)
    if(Python3_Interpreter_FOUND)
        add_test(
            NAME validate_config_cli
            COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/tools/validate_config.py ${CMAKE_SOURCE_DIR}/config/config.ini
        )
        add_test(
            NAME python_offline_tools
            COMMAND ${Python3_EXECUTABLE} -m unittest tests.test_validate_config tests.test_replay_truth_capture tests.test_airframe_tools
            WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        )
    endif()

    find_program(Node_EXECUTABLE node)
    if(Node_EXECUTABLE)
        add_test(
            NAME config_editor_js
            COMMAND ${Node_EXECUTABLE} ${CMAKE_SOURCE_DIR}/tests/test_config_editor.js
            WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        )
    endif()
endif()
message(STATUS "px4xplane v${PROJECT_VERSION} - Cross-Platform Build")
message(STATUS "==========================================================")
message(STATUS "Platform:        ${PLUGIN_PLATFORM}")
message(STATUS "Compiler:        ${CMAKE_CXX_COMPILER_ID}")
message(STATUS "Build Type:      ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ Standard:    C++${CMAKE_CXX_STANDARD}")
message(STATUS "Output:          ${OUTPUT_DIR}/${PLUGIN_PLATFORM}${PLUGIN_EXT}")
message(STATUS "==========================================================")
