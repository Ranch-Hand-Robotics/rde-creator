cmake_minimum_required(VERSION 3.8)
project({{package_name}})

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Create ament index resource which references the libraries in the binary dir
set(node_plugins "")

# Include directories
include_directories(include)

# Create the composable node library
add_library({{node_name}}_component SHARED
  src/node.cpp
)

# Set target properties
set_target_properties({{node_name}}_component PROPERTIES
  VERSION {{package_version}}
  SOVERSION 0
)

# Link dependencies
{{#if support_humble}}
# Runtime check for ROS distribution to choose appropriate CMake approach
set(ROS_DISTRO $ENV{ROS_DISTRO})
if(ROS_DISTRO STREQUAL "humble" OR ROS_DISTRO STREQUAL "galactic" OR ROS_DISTRO STREQUAL "foxy")
  message(STATUS "Using legacy ament_target_dependencies for ROS 2 ${ROS_DISTRO}")
  ament_target_dependencies({{node_name}}_component
    rclcpp
    rclcpp_components
    std_msgs
    {{#if include_lifecycle}}
    rclcpp_lifecycle
    {{/if}}
    {{#if include_service}}
    std_srvs
    {{/if}}
  )
else()
  message(STATUS "Using modern target_link_libraries for ROS 2 ${ROS_DISTRO}")
{{/if}}
{{^support_humble}}
message(STATUS "Using modern target_link_libraries (Humble support disabled)")
{{/support_humble}}
{{#unless (and support_humble (eq ROS_DISTRO "humble"))}}
target_link_libraries({{node_name}}_component PUBLIC
  ${{std_msgs_TARGETS}}
  rclcpp::rclcpp
  rclcpp_components::component
  rclcpp_components::component_manager
  {{#if include_lifecycle}}
  ${{rclcpp_lifecycle_TARGETS}}
  {{/if}}
  {{#if include_service}}
  ${{std_srvs_TARGETS}}
  {{/if}}
)
{{/unless}}
{{#if support_humble}}
endif()
{{/if}}

# Register the component
rclcpp_components_register_nodes({{node_name}}_component "{{package_name}}::{{node_name|pascalcase}}")

# Install the library
install(TARGETS
  {{node_name}}_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install include files
install(DIRECTORY include/
  DESTINATION include
)

# Install launch files
{{#if include_launch}}
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
{{/if}}

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # Add unit tests
  ament_add_gtest(test_{{package_name}}_node
    test/test_{{package_name}}_node.cpp
  )
  
  if(TARGET test_{{package_name}}_node)
    
    {{#if support_humble}}
    # Runtime check for ROS distribution to choose appropriate CMake approach for tests
    if(ROS_DISTRO STREQUAL "humble" OR ROS_DISTRO STREQUAL "galactic" OR ROS_DISTRO STREQUAL "foxy")
      target_link_libraries(test_{{package_name}}_node {{node_name}}_component)
      # Legacy approach for ROS 2 Humble and earlier
      ament_target_dependencies(test_{{package_name}}_node
        rclcpp
        std_msgs
        {{#if include_service}}
        std_srvs
        {{/if}}
      )
    else()
      # Modern CMake approach for ROS 2 Iron, Jazzy, Kilted and later
{{/if}}
{{^support_humble}}
    # Modern CMake approach (support_humble disabled)
{{/support_humble}}
{{#unless (and support_humble (eq ROS_DISTRO "humble"))}}
      target_link_libraries(test_{{package_name}}_node
        ${{std_msgs_TARGETS}}
        rclcpp::rclcpp
        rclcpp_components::component
        rclcpp_components::component_manager
        {{#if include_service}}
        ${{std_srvs_TARGETS}}
        {{/if}}
      )
{{/unless}}
{{#if support_humble}}
    endif()
{{/if}}
  endif()
endif()

ament_package()