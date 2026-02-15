set(MODULE_NAME logger_basic)

file(GLOB_RECURSE SOURCES src/*.cpp)

add_library(${MODULE_NAME} SHARED ${SOURCES})

target_include_directories(
  ${MODULE_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                        $<INSTALL_INTERFACE:include> ${EIGEN3_INCLUDE_DIRS})

target_link_libraries(${MODULE_NAME} PUBLIC rerun_sdk)

add_library(logger_helper_rerun::${MODULE_NAME} ALIAS ${MODULE_NAME})

install(
  TARGETS ${MODULE_NAME}
  EXPORT logger_helper_rerun-targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include)

install(
  DIRECTORY include/
  DESTINATION include
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN "*.hpp")

install(
  EXPORT logger_helper_rerun-targets
  FILE logger_helper_rerun-targets.cmake
  NAMESPACE logger_helper_rerun::
  DESTINATION lib/cmake/logger_helper_rerun)
