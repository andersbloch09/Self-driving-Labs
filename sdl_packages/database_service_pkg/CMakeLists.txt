cmake_minimum_required(VERSION 3.8)
project(database_service_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(PkgConfig REQUIRED)

# Find PostgreSQL libraries using pkg-config
pkg_check_modules(PQXX REQUIRED libpqxx)
pkg_check_modules(PQ REQUIRED libpq)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetChemical.srv"
  "srv/GetOTTraySlot.srv"
  DEPENDENCIES 
    std_msgs
)

# Add library for database functions
add_library(database_functions SHARED src/database_functions.cpp)
target_include_directories(database_functions PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PQXX_INCLUDE_DIRS} 
  ${PQ_INCLUDE_DIRS}
)
ament_target_dependencies(database_functions rclcpp std_msgs)
target_link_libraries(database_functions ${PQXX_LIBRARIES} ${PQ_LIBRARIES})

# Add executables
add_executable(database_service src/database_service.cpp)
add_executable(database_client src/database_client.cpp)
add_executable(interactive_database_test src/interactive_database_test.cpp)

# Add dependencies for executables
ament_target_dependencies(database_service rclcpp std_msgs)
ament_target_dependencies(database_client rclcpp std_msgs)
ament_target_dependencies(interactive_database_test rclcpp std_msgs)

# Link PostgreSQL libraries
target_link_libraries(database_service ${PQXX_LIBRARIES} ${PQ_LIBRARIES})
target_link_libraries(database_client ${PQXX_LIBRARIES} ${PQ_LIBRARIES})
target_link_libraries(interactive_database_test database_functions)

# Add include directories
target_include_directories(database_service PRIVATE ${PQXX_INCLUDE_DIRS} ${PQ_INCLUDE_DIRS})
target_include_directories(database_client PRIVATE ${PQXX_INCLUDE_DIRS} ${PQ_INCLUDE_DIRS})

# Link the generated interfaces to the executables
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(database_service "${cpp_typesupport_target}")
target_link_libraries(database_client "${cpp_typesupport_target}")

# Install library and executables
install(TARGETS
  database_functions
  EXPORT database_functionsTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(TARGETS
  database_service
  database_client
  interactive_database_test
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include
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

ament_export_dependencies(rosidl_default_runtime)
ament_export_targets(database_functionsTargets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_package()
