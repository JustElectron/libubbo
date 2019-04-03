# After running "(sudo) make install"
# libubbo can be found using find_package()

# Libcreate was used as aid for setting up this file.
# https://github.com/AutonomyLab/libcreate

cmake_minimum_required(VERSION 3.5.1)
project(libubbo)

set(PACKAGE_VERSION 0.0.1)

option(LIBUBBO_BUILD_EXAMPLES "Enable the build of examples." ON)

# Set c++ standard to c++11
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# Set output directory for build files
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

#
# Use GNU standard installation directories. CMake will use /usr/local
# as the default install directory. Users may override this by setting
# CMAKE_INSTALL_PREFIX. For example:
#
# cd build && cmake -DCMAKE_INSTALL_PREFIX=/usr ..
#
INCLUDE(GNUInstallDirs)

# Find serial library
# https://github.com/wjwwood/serial
find_package(serial REQUIRED)

#########
# Build #
#########

set(LIBRARY_NAME ubbo)

# Specify location of header files
include_directories(
    include
)

# Declare libraru
add_library(
    ${LIBRARY_NAME}	STATIC
    src/hello.cpp
    src/ubbo.cpp
)

# Link to serial
target_link_libraries(${LIBRARY_NAME}
  ${serial_LIBRARIES}
)

# Build examples
if(LIBUBBO_BUILD_EXAMPLES)
# Declare example executables
set(EXAMPLES
    hello_world
    Serial_test
)

foreach(EXAMPLE ${EXAMPLES})
# adds executable for example
add_executable(
    ${EXAMPLE}     examples/${EXAMPLE}.cpp
)

# links example to libubbo
target_link_libraries(${EXAMPLE}
    ${LIBRARY_NAME}
    )
endforeach()
endif()

#################
# Configuration #
#################

# Install directories layout:
#   * <prefix>/lib/
#   * <prefix>/bin/
#   * <prefix>/include/
#   * <prefix>/lib/cmake/<PROJECT-NAME>
#   * <prefix>/share/<PROJECT_NAME>
set(LIB_INSTALL_DIR "lib")
set(BIN_INSTALL_DIR "bin")
set(INCLUDE_INSTALL_DIR "include")
set(CONFIG_INSTALL_DIR "${LIB_INSTALL_DIR}/cmake/${PROJECT_NAME}")
set(SHARE_INSTALL_DIR "share/${PROJECT_NAME}")

set(GENERATED_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated")
set(VERSION_CONFIG "${GENERATED_DIR}/${PROJECT_NAME}-config-version.cmake")
set(PROJECT_CONFIG "${GENERATED_DIR}/${PROJECT_NAME}-config.cmake")
set(TARGETS_EXPORT_NAME "${PROJECT_NAME}-targets")

include(CMakePackageConfigHelpers)

# Configure '<PROJECT-NAME>-config-version.cmake'
write_basic_package_version_file(
  "${VERSION_CONFIG}"
  VERSION "${PACKAGE_VERSION}"
  COMPATIBILITY SameMajorVersion
)

# Configure '<PROJECT-NAME>-config.cmake'
configure_package_config_file(
  "config.cmake.in"
  "${PROJECT_CONFIG}"
  INSTALL_DESTINATION "${CONFIG_INSTALL_DIR}"
  PATH_VARS
    INCLUDE_INSTALL_DIR
    LIBRARY_NAME
)

###########
# Install #
###########

# Install targets
install(
  TARGETS ${LIBRARY_NAME}
  EXPORT "${TARGETS_EXPORT_NAME}"
  LIBRARY DESTINATION "${LIB_INSTALL_DIR}"
  ARCHIVE DESTINATION "${LIB_INSTALL_DIR}"
  RUNTIME DESTINATION "${BIN_INSTALL_DIR}"
  INCLUDES DESTINATION "${INCLUDE_INSTALL_DIR}"
)

# Install headers
install(
  DIRECTORY include/
  DESTINATION ${INCLUDE_INSTALL_DIR}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

# Install config
install(
  FILES "${PROJECT_CONFIG}" "${VERSION_CONFIG}"
  DESTINATION "${CONFIG_INSTALL_DIR}"
)

# Install targets
install(
  EXPORT "${TARGETS_EXPORT_NAME}"
  DESTINATION "${CONFIG_INSTALL_DIR}"
)