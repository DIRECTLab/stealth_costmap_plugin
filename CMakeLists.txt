cmake_minimum_required(VERSION 3.8)
project(audio_costmap_plugin)

set(lib_name ${PROJECT_NAME}_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# === Dependencies ===
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)

set(dep_pkgs
    rclcpp
    nav2_costmap_2d
    pluginlib)


# === Build ===

add_library(${lib_name} SHARED
    src/audio_layer.cpp
    src/audio_map.cpp)
include_directories(include)


# === Installation ===

install(TARGETS ${lib_name}
        DESTINATION lib)


# === Ament Work ===

# pluginlib_export_plugin_description_file() installs stealth_layer.xml
# file into "share" directory and sets ament indexes for it.
# This allows the plugin to be discovered as a plugin of required type.
pluginlib_export_plugin_description_file(nav2_costmap_2d audio_layer.xml)
ament_target_dependencies(${lib_name} ${dep_pkgs})
ament_package()
