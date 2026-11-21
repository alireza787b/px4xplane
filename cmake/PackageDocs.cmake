cmake_minimum_required(VERSION 3.15)

if(NOT DEFINED SOURCE_ROOT OR NOT DEFINED PLUGIN_ROOT)
    message(FATAL_ERROR "SOURCE_ROOT and PLUGIN_ROOT are required")
endif()

# Keep one card location, including when refreshing an older build directory.
foreach(name ALIA EHANG184 QUADTAILSITTER CESSNA172 TB2)
    file(REMOVE "${PLUGIN_ROOT}/${name}_XPLANE12_TEST.md")
endforeach()
file(REMOVE "${PLUGIN_ROOT}/compact-qtailsitter.md")

foreach(name LICENSE CHANGELOG.md)
    configure_file("${SOURCE_ROOT}/${name}" "${PLUGIN_ROOT}/${name}" COPYONLY)
endforeach()
configure_file("${SOURCE_ROOT}/docs/guided-launcher.md"
    "${PLUGIN_ROOT}/docs/guided-launcher.md" COPYONLY)

# Re-read source Markdown each time so refreshes cannot compound substitutions.
file(GLOB_RECURSE pages RELATIVE "${SOURCE_ROOT}/docs" "${SOURCE_ROOT}/docs/*.md")
foreach(page IN LISTS pages)
    file(READ "${SOURCE_ROOT}/docs/${page}" content)
    string(REPLACE "](../config/px4_params/" "](../px4_airframes/" content "${content}")
    file(WRITE "${PLUGIN_ROOT}/docs/${page}" "${content}")
endforeach()

file(READ "${SOURCE_ROOT}/config/px4_params/README.md" content)
string(REPLACE "](../../docs/" "](../docs/" content "${content}")
file(WRITE "${PLUGIN_ROOT}/px4_airframes/README.md" "${content}")
