# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_agv_cpp_examples_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED agv_cpp_examples_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(agv_cpp_examples_FOUND FALSE)
  elseif(NOT agv_cpp_examples_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(agv_cpp_examples_FOUND FALSE)
  endif()
  return()
endif()
set(_agv_cpp_examples_CONFIG_INCLUDED TRUE)

# output package information
if(NOT agv_cpp_examples_FIND_QUIETLY)
  message(STATUS "Found agv_cpp_examples: 0.0.0 (${agv_cpp_examples_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'agv_cpp_examples' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${agv_cpp_examples_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(agv_cpp_examples_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${agv_cpp_examples_DIR}/${_extra}")
endforeach()
