cmake_minimum_required(VERSION 3.16)
project(epmc_hardware_interface LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


## COMPILE
add_library(
  epmc_hardware_interface
  SHARED
  src/epmc_hardware_interface.cpp
)

target_compile_features(epmc_hardware_interface PUBLIC cxx_std_17)

target_include_directories(epmc_hardware_interface PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

ament_target_dependencies(
  epmc_hardware_interface PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_link_libraries(epmc_hardware_interface PUBLIC serial)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface epmc_hardware_interface.xml)

# INSTALL
install(
  DIRECTORY include/
  DESTINATION include/epmc_hardware_interface
)

install(TARGETS epmc_hardware_interface
  EXPORT export_epmc_hardware_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

## EXPORTS
ament_export_targets(export_epmc_hardware_interface HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
