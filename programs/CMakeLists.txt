# Build PARS factor extensions as an object library
add_library(pars_factors OBJECT
    extensions/PARSRangeFactor.cpp
    extensions/PARSAzimuthFactor.cpp
    extensions/PARSElevationFactor.cpp
)
target_include_directories(pars_factors PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/extensions
)
target_link_libraries(pars_factors PRIVATE gtsam)

# Build each program .cpp as a separate executable
file(GLOB program_files "*.cpp")
foreach(program_file ${program_files})
    get_filename_component(program_name ${program_file} NAME_WE)
    add_executable(${program_name} ${program_file} $<TARGET_OBJECTS:pars_factors>)
    target_include_directories(${program_name} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/extensions
    )
    target_link_libraries(${program_name} gtsam gtsam_unstable)
    gtsam_apply_build_flags(${program_name})
endforeach()
