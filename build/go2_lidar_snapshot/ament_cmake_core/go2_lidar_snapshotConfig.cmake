# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_go2_lidar_snapshot_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED go2_lidar_snapshot_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(go2_lidar_snapshot_FOUND FALSE)
  elseif(NOT go2_lidar_snapshot_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(go2_lidar_snapshot_FOUND FALSE)
  endif()
  return()
endif()
set(_go2_lidar_snapshot_CONFIG_INCLUDED TRUE)

# output package information
if(NOT go2_lidar_snapshot_FIND_QUIETLY)
  message(STATUS "Found go2_lidar_snapshot: 0.0.1 (${go2_lidar_snapshot_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'go2_lidar_snapshot' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${go2_lidar_snapshot_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(go2_lidar_snapshot_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${go2_lidar_snapshot_DIR}/${_extra}")
endforeach()
