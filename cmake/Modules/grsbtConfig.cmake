INCLUDE(FindPkgConfig)
PKG_CHECK_MODULES(PC_GRSBT grsbt)

FIND_PATH(
    GRSBT_INCLUDE_DIRS
    NAMES grsbt/api.h
    HINTS $ENV{GRSBT_DIR}/include
        ${PC_GRSBT_INCLUDEDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/include
          /usr/local/include
          /usr/include
)

FIND_LIBRARY(
    GRSBT_LIBRARIES
    NAMES gnuradio-grsbt
    HINTS $ENV{GRSBT_DIR}/lib
        ${PC_GRSBT_LIBDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/lib
          ${CMAKE_INSTALL_PREFIX}/lib64
          /usr/local/lib
          /usr/local/lib64
          /usr/lib
          /usr/lib64
)

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(GRSBT DEFAULT_MSG GRSBT_LIBRARIES GRSBT_INCLUDE_DIRS)
MARK_AS_ADVANCED(GRSBT_LIBRARIES GRSBT_INCLUDE_DIRS)

