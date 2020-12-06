cmake_minimum_required(VERSION 3.0.2)
project(rviz_winter)
set_directory_properties(PROPERTIES COMPILE_OPTIONS "-std=c++11;-Wall;-Werror")

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    pluginlib
    rviz
)

set(CMAKE_AUTOMOC ON)

find_package(Qt5 REQUIRED COMPONENTS
  Core
  Widgets
)
set(QT_LIBRARIES Qt5::Widgets)

add_definitions(-DQT_NO_KEYWORDS)

catkin_package(
    CATKIN_DEPENDS
        geometry_msgs
        pluginlib
        rviz
    LIBRARIES winter_display
    INCLUDE_DIRS include
)

include_directories(include ${catkin_INCLUDE_DIRS})

qt5_wrap_cpp(MOC_FILES
  include/rviz_winter/winter_display.h
)

add_library(winter_display
    src/winter_display.cpp
    ${MOC_FILES}
)
add_dependencies(winter_display ${catkin_EXPORTED_TARGETS})
target_link_libraries(winter_display ${QT_LIBRARIES} ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS roslaunch roslint)
  roslint_cpp()
  roslint_add_test()
  roslaunch_add_file_check(launch)
endif()

install(TARGETS winter_display
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(FILES rviz_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES icons/classes/Winter.png
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/icons/classes
)
