cmake_minimum_required(VERSION 3.8)
project(tdt_nav_kit)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV QUIET)
find_package(OsqpEigen QUIET)
find_package(rclcpp REQUIRED)

set(TDT_NAV_KIT_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_library(tdt_nav_frontend SHARED
  src/YAstar/yastar.cpp
  src/YAstar/kinodynamicAstar.cpp
)
target_include_directories(tdt_nav_frontend PUBLIC
  $<BUILD_INTERFACE:${TDT_NAV_KIT_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(tdt_nav_frontend PUBLIC Eigen3::Eigen)
if(OpenCV_FOUND)
  target_include_directories(tdt_nav_frontend PUBLIC ${OpenCV_INCLUDE_DIRS})
  target_compile_definitions(tdt_nav_frontend PUBLIC TDT_NAV_KIT_OPENCV_ENABLED)
  target_link_libraries(tdt_nav_frontend PUBLIC ${OpenCV_LIBS})
endif()

install(DIRECTORY include/ DESTINATION include)
install(TARGETS tdt_nav_frontend
  EXPORT tdt_nav_kit_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(OsqpEigen_FOUND)
  add_library(tdt_nav_trajectory SHARED
    src/MinimumSnapOsqp/minimumSnap.cpp
    src/MinimumSnapOsqp/sfcSquare.cpp
  )
  target_include_directories(tdt_nav_trajectory PUBLIC
    $<BUILD_INTERFACE:${TDT_NAV_KIT_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(tdt_nav_trajectory PUBLIC Eigen3::Eigen OsqpEigen::OsqpEigen)
  if(OpenCV_FOUND)
    target_include_directories(tdt_nav_trajectory PUBLIC ${OpenCV_INCLUDE_DIRS})
    target_compile_definitions(tdt_nav_trajectory PUBLIC TDT_NAV_KIT_OPENCV_ENABLED)
    target_link_libraries(tdt_nav_trajectory PUBLIC ${OpenCV_LIBS})
  endif()
  install(TARGETS tdt_nav_trajectory
    EXPORT tdt_nav_kit_targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
else()
  message(STATUS "OsqpEigen not found: building the YAstar front-end only")
endif()

ament_export_targets(tdt_nav_kit_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 geometry_msgs nav_msgs rclcpp)

add_executable(tdt_nav_kit_demo src/tdt_nav_kit_demo_node.cpp)
target_link_libraries(tdt_nav_kit_demo tdt_nav_frontend)
ament_target_dependencies(tdt_nav_kit_demo geometry_msgs nav_msgs rclcpp)
install(TARGETS tdt_nav_kit_demo DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
ament_package()
