# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_MA_susbriber_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED MA_susbriber_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(MA_susbriber_FOUND FALSE)
  elseif(NOT MA_susbriber_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(MA_susbriber_FOUND FALSE)
  endif()
  return()
endif()
set(_MA_susbriber_CONFIG_INCLUDED TRUE)

# output package information
if(NOT MA_susbriber_FIND_QUIETLY)
  message(STATUS "Found MA_susbriber: 0.0.0 (${MA_susbriber_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'MA_susbriber' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${MA_susbriber_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(MA_susbriber_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${MA_susbriber_DIR}/${_extra}")
endforeach()
