add_executable(logger_helper_rerun_example example_log_msg.cpp)
target_link_libraries(logger_helper_rerun_example
                      PRIVATE logger_helper_rerun::logger_basic)

add_executable(logger_helper_rerun_2d_example example_2d_msg.cpp)
target_link_libraries(
  logger_helper_rerun_2d_example PRIVATE logger_helper_rerun::logger_basic
                                         logger_helper_rerun::logger_2d)
target_compile_definitions(
  logger_helper_rerun_2d_example
  PRIVATE "EXAMPLE_INPUT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}\"")

add_executable(logger_helper_rerun_3d_example example_3d_msg.cpp)
target_link_libraries(
  logger_helper_rerun_3d_example PRIVATE logger_helper_rerun::logger_basic
                                         logger_helper_rerun::logger_3d)
