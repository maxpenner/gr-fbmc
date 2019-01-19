INCLUDE(FindPkgConfig)
PKG_CHECK_MODULES(PC_FBMC1 fbmc1)

FIND_PATH(
    FBMC1_INCLUDE_DIRS
    NAMES fbmc1/api.h
    HINTS $ENV{FBMC1_DIR}/include
        ${PC_FBMC1_INCLUDEDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/include
          /usr/local/include
          /usr/include
)

FIND_LIBRARY(
    FBMC1_LIBRARIES
    NAMES gnuradio-fbmc1
    HINTS $ENV{FBMC1_DIR}/lib
        ${PC_FBMC1_LIBDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/lib
          ${CMAKE_INSTALL_PREFIX}/lib64
          /usr/local/lib
          /usr/local/lib64
          /usr/lib
          /usr/lib64
)

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(FBMC1 DEFAULT_MSG FBMC1_LIBRARIES FBMC1_INCLUDE_DIRS)
MARK_AS_ADVANCED(FBMC1_LIBRARIES FBMC1_INCLUDE_DIRS)

