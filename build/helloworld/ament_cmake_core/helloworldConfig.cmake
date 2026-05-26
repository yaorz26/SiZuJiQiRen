# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_helloworld_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED helloworld_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(helloworld_FOUND FALSE)
  elseif(NOT helloworld_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(helloworld_FOUND FALSE)
  endif()
  return()
endif()
set(_helloworld_CONFIG_INCLUDED TRUE)

# output package information
if(NOT helloworld_FIND_QUIETLY)
  message(STATUS "Found helloworld: 0.0.0 (${helloworld_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'helloworld' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${helloworld_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(helloworld_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${helloworld_DIR}/${_extra}")
endforeach()
