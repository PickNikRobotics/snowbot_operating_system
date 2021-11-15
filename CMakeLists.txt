cmake_minimum_required(VERSION 3.5)
project(snowbot_operating_system)
set_directory_properties(PROPERTIES COMPILE_OPTIONS "-Wall;-Werror")

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)

set(CMAKE_AUTOMOC ON)

find_package(Qt5 REQUIRED COMPONENTS
  Core
  Widgets
)
set(QT_LIBRARIES Qt5::Widgets)

add_definitions(-DQT_NO_KEYWORDS)

qt5_wrap_cpp(MOC_FILES
  include/snowbot_operating_system/snow_display.h
)

add_library(snow_display
    src/snow_display.cpp
    ${MOC_FILES}
)
target_include_directories(snow_display PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(snow_display geometry_msgs pluginlib rviz_common rviz_rendering)

if(BUILD_TESTING)
  find_package(ament_cmake_clang_format REQUIRED)
  ament_clang_format(CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/.clang-format")
endif()

install(TARGETS snow_display
        EXPORT export_snow_display
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION include/${PROJECT_NAME}
)
install(FILES rviz_plugins.xml
        DESTINATION share/${PROJECT_NAME}
)
install(FILES icons/classes/Snow.png
        DESTINATION share/${PROJECT_NAME}/icons/classes
)
install(DIRECTORY include/
        DESTINATION include
)
ament_export_include_directories(include)
ament_export_libraries(snow_display)
ament_export_dependencies(geometry_msgs pluginlib rviz_common)
ament_export_targets(export_snow_display)
pluginlib_export_plugin_description_file(rviz_common rviz_plugins.xml)
ament_package()
