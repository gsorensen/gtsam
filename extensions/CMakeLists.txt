# parnav extensions: PARS measurement factors and preintegration helpers.
# Built as an OBJECT library so downstream targets can link the object files
# directly without producing a standalone library artifact.

add_library(parnav_extensions OBJECT
    RangeFactor.cpp
    AzimuthFactor.cpp
    ElevationFactor.cpp
    BaroFactor.cpp
    CompassFactor.cpp
    GNSSAttFactor.cpp
    PreintegrationHelpers.cpp
)

target_include_directories(parnav_extensions PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(parnav_extensions PUBLIC gtsam)

gtsam_apply_build_flags(parnav_extensions)
