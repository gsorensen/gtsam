# Build each program .cpp as a separate executable.
# Programs link against the parnav_extensions OBJECT library (defined in
# the top-level extensions/ directory) and gtsam / gtsam_unstable.

file(GLOB program_files "*.cpp")
foreach(program_file ${program_files})
    get_filename_component(program_name ${program_file} NAME_WE)
    add_executable(${program_name}
        ${program_file}
        $<TARGET_OBJECTS:parnav_extensions>
    )
    target_include_directories(${program_name} PRIVATE
        ${CMAKE_SOURCE_DIR}/extensions
    )
    target_link_libraries(${program_name} gtsam gtsam_unstable)
    gtsam_apply_build_flags(${program_name})
endforeach()
