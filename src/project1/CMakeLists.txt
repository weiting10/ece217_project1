cmake_minimum_required(VERSION 3.10)
project(project1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Boost COMPONENTS program_options REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/PlanningQuery.srv"
  DEPENDENCIES geometry_msgs nav_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp
)

include_directories(
  include
  ${Boost_INCLUDE_DIR}
)

add_library(project1_library
  src/${PROJECT_NAME}/search_node.cpp
  src/test_search_node.cpp
  src/planning_rrt.cpp
)
ament_target_dependencies(project1_library rclcpp geometry_msgs nav_msgs std_msgs visualization_msgs)


set(dependencies "rclcpp" "pluginlib" "builtin_interfaces" "std_msgs" "geometry_msgs" "nav_msgs" "visualization_msgs")

set(libraries "${Boost_PROGRAM_OPTIONS_LIBRARY}" "${Boost_SYSTEM_LIBRARY}" "${Boost_FILESYSTEM_LIBRARY}" "${ament_LIBRARIES}" "${cpp_typesupport_target}")


add_executable(test_node src/test_node.cpp)
target_include_directories(test_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(test_node PUBLIC c_std_99 cxx_std_17) 

add_executable(test_publisher src/test_publisher.cpp)
target_include_directories(test_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(test_publisher PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(test_publisher rclcpp geometry_msgs)
target_link_libraries(test_publisher ${Boost_PROGRAM_OPTIONS_LIBRARY})

add_executable(test_subscriber src/test_subscriber.cpp)
target_include_directories(test_subscriber PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(test_subscriber PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(test_subscriber rclcpp geometry_msgs)
target_link_libraries(test_subscriber ${Boost_PROGRAM_OPTIONS_LIBRARY})

add_executable(test_server src/test_server.cpp 
			   src/project1/search_node.cpp 
			   src/test_search_node.cpp
			   src/planning_rrt.cpp
)
target_include_directories(test_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(test_server PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(test_server rclcpp geometry_msgs nav_msgs visualization_msgs)
target_link_libraries(test_server ${Boost_PROGRAM_OPTIONS_LIBRARY} ${cpp_typesupport_target} project1_library)

add_executable(test_client src/test_client.cpp)
target_include_directories(test_client PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(test_client PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(test_client rclcpp geometry_msgs nav_msgs visualization_msgs)
target_link_libraries(test_client ${Boost_PROGRAM_OPTIONS_LIBRARY} ${cpp_typesupport_target})


install(TARGETS 
	test_node
	test_publisher
	test_subscriber
	test_server
	test_client
        DESTINATION lib/${PROJECT_NAME})

install(TARGETS project1_library
	EXPORT 
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib
	RUNTIME DESTINATION bin
	DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(${dependencies} Boost rosidl_defaul_runtime)
ament_export_include_directories(include)
#ament_export_libraries(ece217)

#ament_export_targets(ece217-targets HAS_LIBRARY_TARGET)

ament_package()
