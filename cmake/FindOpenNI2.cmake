###############################################################################
# Find OpenNI2
#
#     find_package(OpenNI2)
#
# Variables defined by this module:
#
#  OpenNI2_FOUND               True if OpenNI2 was found
#  OpenNI2_INCLUDE_DIRSS       The location(s) of OpenNI2 headers
#  OpenNI2_LIBRARIES           Libraries needed to use OpenNI2
#  OpenNI2_DEFINITIONS         Compiler flags for OpenNI2

find_package(PkgConfig QUIET)

# Find LibUSB
if(NOT WIN32)
  pkg_check_modules(PC_USB_10 libusb-1.0)
  find_path(USB_10_INCLUDE_DIR libusb-1.0/libusb.h
            HINTS ${PC_USB_10_INCLUDEDIR} ${PC_USB_10_INCLUDE_DIRS} "${USB_10_ROOT}" "$ENV{USB_10_ROOT}"
            PATH_SUFFIXES libusb-1.0)

  find_library(USB_10_LIBRARY
               NAMES usb-1.0
               HINTS ${PC_USB_10_LIBDIR} ${PC_USB_10_LIBRARY_DIRS} "${USB_10_ROOT}" "$ENV{USB_10_ROOT}"
               PATH_SUFFIXES lib)

  include(FindPackageHandleStandardArgs)
  find_package_handle_standard_args(USB_10 DEFAULT_MSG USB_10_LIBRARY USB_10_INCLUDE_DIR)

  if(NOT USB_10_FOUND)
    message(STATUS "OpenNI 2 disabled because libusb-1.0 not found.")
    return()
  else()
    include_directories(SYSTEM ${USB_10_INCLUDE_DIR})
  endif()
endif(NOT WIN32)

if(${CMAKE_VERSION} VERSION_LESS 2.8.2)
  pkg_check_modules(PC_OPENNI2 libopenni2)
else()
  pkg_check_modules(PC_OPENNI2 QUIET libopenni2)
endif()

set(OpenNI2_DEFINITIONS ${PC_OPENNI_CFLAGS_OTHER})

set(OPENNI2_SUFFIX)
if(WIN32 AND CMAKE_SIZEOF_VOID_P EQUAL 8)
  set(OPENNI2_SUFFIX 64)
endif(WIN32 AND CMAKE_SIZEOF_VOID_P EQUAL 8)

find_path(OpenNI2_INCLUDE_DIRS OpenNI.h
          PATHS "$ENV{OPENNI2_INCLUDE${OPENNI2_SUFFIX}}"  # Win64 needs '64' suffix
                "/usr/include/openni2"                    # common path for deb packages
          PATH_SUFFIXES include/openni2
)

find_library(OpenNI2_LIBRARIES
             NAMES OpenNI2      # No suffix needed on Win64
                   libOpenNI2   # Linux
             PATHS "$ENV{OPENNI2_LIB${OPENNI2_SUFFIX}}"   # Windows default path, Win64 needs '64' suffix
                   "$ENV{OPENNI2_REDIST}"                 # Linux install does not use a separate 'lib' directory
)

if(OpenNI2_INCLUDE_DIRS AND OpenNI2_LIBRARIES)

  # Include directories
  set(OpenNI2_INCLUDE_DIRSS ${OpenNI2_INCLUDE_DIRS})
  unset(OpenNI2_INCLUDE_DIRS)
  mark_as_advanced(OpenNI2_INCLUDE_DIRSS)

  # Libraries
  if(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
    set(OpenNI2_LIBRARIES ${OpenNI2_LIBRARIES} ${LIBUSB_1_LIBRARIES})
  else()
    set(OpenNI2_LIBRARIES ${OpenNI2_LIBRARIES})
  endif()
  unset(OpenNI2_LIBRARIES)
  mark_as_advanced(OpenNI2_LIBRARIES)

  set(OPENNI2_REDIST_DIR $ENV{OPENNI2_REDIST${OPENNI2_SUFFIX}})
  mark_as_advanced(OPENNI2_REDIST_DIR)

endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(OpenNI2
  FOUND_VAR OpenNI2_FOUND
  REQUIRED_VARS OpenNI2_LIBRARIES OpenNI2_INCLUDE_DIRSS
)

if(OpenNI2_FOUND)
  message(STATUS "OpenNI2 found (include: ${OpenNI2_INCLUDE_DIRSS}, lib: ${OpenNI2_LIBRARIES})")
endif()
