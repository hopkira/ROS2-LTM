cmake_minimum_required(VERSION 3.5)
project(ltm_package)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# services and clients
ament_target_dependencies(service_ltm rclpy std_msgs example_interfaces)
ament_target_dependencies(ltm_test_client rclpy std_msgs example_interfaces)

install(TARGETS
  service_ltm
  ltm_test_client
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_add_gtest(${PROJECT_NAME} test/test_ltm.py)
  ament_target_dependencies(test_ltm rclpy std_msgs example_interfaces)
endif()

if(BUILD_TESTING AND NOT WIN32)
  find_package(Python3 COMPONENTS Interpreter)
  # the Python executable can be different on each platform
  execute_process(COMMAND "${Python3_EXECUTABLE}" -m compileall .
    RESULT_VARIABLE HAD_ERROR)
  if(NOT ${HAD_ERROR} EQUAL "0")
    message(FATAL_ERROR "Failed to run test code style checks.")
  endif()
endif()

ament_package()