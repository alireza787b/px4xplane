#==============================================================================
# px4xplane - Cross-Platform CMake Build System
# X-Plane Plugin for PX4 SITL Integration
# Repository: alireza787b/px4xplane
#
# Supports: Windows (MSVC/MinGW), Linux (GCC/Clang), macOS (Clang)
# Build: mkdir build && cd build && cmake .. && cmake --build .
#==============================================================================

cmake_minimum_required(VERSION 3.15)
project(px4xplane VERSION 2.5.2 LANGUAGES CXX)

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
set(OUTPUT_DIR "${CMAKE_BINARY_DIR}/${PLUGIN_PLATFORM}/${CMAKE_BUILD_TYPE}/px4xplane/64")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${OUTPUT_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${OUTPUT_DIR})

#==============================================================================
# Source Files
#==============================================================================

set(SOURCES
    src/ConfigManager.cpp
    src/configReader.cpp
    src/ConnectionManager.cpp
    src/ConnectionStatusHUD.cpp
    src/DataRefManager.cpp
    src/MAVLinkManager.cpp
    src/px4xplane.cpp
    src/TimeManager.cpp
    src/UIConstants.cpp
    src/UIHandler.cpp
)

set(HEADERS
    include/ConfigManager.h
    include/configReader.h
    include/ConnectionManager.h
    include/ConnectionStatusHUD.h
    include/DataRefManager.h
    include/MAVLinkManager.h
    include/TimeManager.h
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
        _CRT_SECURE_NO_WARNINGS
        _CRT_NONSTDC_NO_DEPRECATE
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
    set_target_properties(${PROJECT_NAME} PROPERTIES
        LINK_FLAGS "-flat_namespace -undefined suppress"
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
    set_target_properties(${PROJECT_NAME} PROPERTIES
        LINK_FLAGS "-rdynamic -nodefaultlibs -undefined_warning"
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
    target_link_libraries(${PROJECT_NAME} PRIVATE
        "-framework XPLM"
        "-framework XPWidgets"
        "-framework OpenGL"
    )

    # Add framework search path
    target_link_directories(${PROJECT_NAME} PRIVATE ${SDK_LIB_DIR})

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
        ${OUTPUT_DIR}/../config.ini
    COMMENT "Copying config.ini to plugin directory"
)

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${CMAKE_SOURCE_DIR}/README.md
        ${OUTPUT_DIR}/../README.md
    COMMENT "Copying README.md to plugin directory"
)

# Copy PX4 parameter files
file(GLOB PX4_PARAMS "${CMAKE_SOURCE_DIR}/config/px4_params/*")
foreach(PARAM_FILE ${PX4_PARAMS})
    add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${PARAM_FILE}
            ${OUTPUT_DIR}/../
        COMMENT "Copying PX4 parameter: ${PARAM_FILE}"
    )
endforeach()

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
message(STATUS "px4xplane v${PROJECT_VERSION} - Cross-Platform Build")
message(STATUS "==========================================================")
message(STATUS "Platform:        ${PLUGIN_PLATFORM}")
message(STATUS "Compiler:        ${CMAKE_CXX_COMPILER_ID}")
message(STATUS "Build Type:      ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ Standard:    C++${CMAKE_CXX_STANDARD}")
message(STATUS "Output:          ${OUTPUT_DIR}/${PLUGIN_PLATFORM}${PLUGIN_EXT}")
message(STATUS "==========================================================")
