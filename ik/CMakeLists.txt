cmake_minimum_required(VERSION 3.10)
project(ik)

## Use C++14
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
## add_definitions(-Wall -Werror)
add_definitions(-Wall)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# [find dependencies]

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# [build]

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/kdl.cpp
  src/moveit.cpp
  src/test.cpp
  src/joint_broadcaster.cpp
  # src/login_test
)

# [single cpp]
# add_executable(moveit_server_exe src/moveit_server.cpp)
# ament_target_dependencies(
#   moveit_server_exe
#   "moveit_ros_planning_interface"
#   "moveit_visual_tools"
#   "rclcpp"
# )
# install(TARGETS moveit_server_exe
#   DESTINATION lib/${PROJECT_NAME})

# [dep.system.ceres]
find_package(Ceres REQUIRED)
target_include_directories(${PROJECT_NAME} PUBLIC ${CERES_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${CERES_LIBRARIES})

# [dep.system.jsoncpp]
find_package(jsoncpp REQUIRED)
target_include_directories(${PROJECT_NAME} PUBLIC ${jsoncpp_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} jsoncpp)

# [dep.system.Poco]
# find_package(Poco REQUIRED COMPONENTS Foundation Net Util)
# message(STATUS "Poco_FOUND: ${Poco_FOUND}")
# target_include_directories(${PROJECT_NAME} PUBLIC ${Poco_INCLUDE_DIRS})
# target_link_libraries(${PROJECT_NAME} ${Poco_LIBRARIES})

# [dep.local.aubo]
target_include_directories(${PROJECT_NAME} PUBLIC
  ${PROJECT_SOURCE_DIR}/third_party/robotSDK/inc
)
target_link_directories(${PROJECT_NAME} PUBLIC
  ${PROJECT_SOURCE_DIR}/third_party/robotSDK/lib/linux_x64
)
target_link_libraries(${PROJECT_NAME} auborobotcontroller)
install(DIRECTORY ${PROJECT_SOURCE_DIR}/third_party/robotSDK/lib/linux_x64/
  DESTINATION lib
)

# [log4cplus(local)]
target_include_directories(${PROJECT_NAME} PUBLIC
  ${PROJECT_SOURCE_DIR}/third_party/log4cplus/linux_x64/inc
)
target_link_directories(${PROJECT_NAME} PUBLIC
  ${PROJECT_SOURCE_DIR}/third_party/log4cplus/linux_x64/lib
)
target_link_libraries(${PROJECT_NAME} log4cplus)
install(DIRECTORY ${PROJECT_SOURCE_DIR}/third_party/log4cplus/linux_x64/lib/
  DESTINATION lib
)

# [register node]
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN ik::kdl::Ik
  EXECUTABLE ik_kdl_exe
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN ik::moveit::Ik
  EXECUTABLE ik_moveit_exe
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN ik::test::Ik
  EXECUTABLE ik_test_exe
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN ik::joint_broadcaster::JointBroadcaster
  EXECUTABLE ik_joint_broadcaster_exe
)

# rclcpp_components_register_node(${PROJECT_NAME}
#   PLUGIN ik::login_test::LoginTest
#   EXECUTABLE login_test_exe
# )


# [testing]

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_uncrustify
    ament_cmake_cpplint
  )
  ament_lint_auto_find_test_dependencies()

  # find_package(ament_cmake_gtest)
  # set(TEST_NAME test_kalman_filter)
  # ament_add_gtest(${TEST_NAME} test/${TEST_NAME}.cpp)
  # target_link_libraries(${TEST_NAME} ${PROJECT_NAME})
endif()

# [install]

ament_auto_package(
  INSTALL_TO_SHARE
  config
)

