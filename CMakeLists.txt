cmake_minimum_required(VERSION 3.5)
project(kindyn)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosbag2 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

#sudo apt-get install ros-foxy-ros2-control
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_manager REQUIRED)

# eigen_conversions merged in geometry_2 in ros2 https://github.com/ros2/geometry2/pull/311
# apt-get install ros-foxy-geometry2
# find_package(eigen_conversions REQUIRED)
find_package(geometry2 REQUIRED)
#sudo apt-get install ros-foxy-effort-controllers
find_package(effort_controllers REQUIRED)

find_package(roboy_control_msgs REQUIRED)
find_package(roboy_middleware_msgs REQUIRED)
find_package(roboy_simulation_msgs REQUIRED)
find_package(robot_state_publisher REQUIRED)


ament_export_dependencies(
  roboy_middleware_msgs
  common_utilities
)


# Eigen
find_package(PkgConfig REQUIRED )
pkg_check_modules(EIGEN3 REQUIRED eigen3)
include_directories(${EIGEN3_INCLUDE_DIRS})

# # libxml++
# find_package(LibXML++ REQUIRED)
find_package (LibXml2 REQUIRED)
include_directories(${LibXml2_INCLUDE_DIRS})

# idyntree
find_package(iDynTree 0.11.1 REQUIRED)
include_directories(${iDynTree_INCLUDE_DIRS})

# qpOASES
#https://github.com/coin-or/qpOASES
#mkdir build && cd build 
#cmake .. 
#sudo make 
#sudo make install  
find_package(qpOASES REQUIRED)
include_directories(${qpOASES_INCLUDE_DIRS})

# find openmp support
find_package(OpenMP)
if (OPENMP_FOUND)
	set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
	set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
	set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()



#==============================================================================
# Application
#==============================================================================
# CARDSflow hardware interface
# add_library(CardsflowHardwareInterface
# 		src/controller/cardsflow_state_interface.cpp
# 		src/controller/cardsflow_command_interface.cpp)
# add_dependencies(CardsflowHardwareInterface roboy_middleware_msgs_generate_messages_cpp)
# target_link_libraries(CardsflowHardwareInterface ${catkin_LIBRARIES})

# # Cable Length Controller
# add_library(CableLengthController src/controller/cableLengthController.cpp )
# add_dependencies(CableLengthController roboy_middleware_msgs_generate_messages_cpp roboy_control_msgs_generate_messages_cpp)
# target_link_libraries(CableLengthController ${catkin_LIBRARIES} CardsflowHardwareInterface)

# # Cable Length Velocity Controller
# add_library(CableLengthVelocityController src/controller/cableLengthVelocityController.cpp )
# add_dependencies(CableLengthVelocityController roboy_middleware_msgs_generate_messages_cpp roboy_control_msgs_generate_messages_cpp)
# target_link_libraries(CableLengthVelocityController ${catkin_LIBRARIES} CardsflowHardwareInterface)

# # Torque Position Controller
# add_library(TorquePositionController src/controller/torquePositionController.cpp )
# add_dependencies(TorquePositionController roboy_middleware_msgs_generate_messages_cpp roboy_control_msgs_generate_messages_cpp)
# target_link_libraries(TorquePositionController ${catkin_LIBRARIES} CardsflowHardwareInterface)

# # Force Position Controller
# add_library(ForcePositionController src/controller/forcePositionController.cpp )
# add_dependencies(ForcePositionController roboy_middleware_msgs_generate_messages_cpp roboy_control_msgs_generate_messages_cpp)
# target_link_libraries(ForcePositionController ${catkin_LIBRARIES} CardsflowHardwareInterface)

# # kindyn library
# add_library(kindyn SHARED
#         src/robot.cpp
# 		src/kinematics.cpp
# 		src/EigenExtension.cpp
# 		include/kindyn/EigenExtension.hpp
# 		include/kindyn/robot.hpp
# 		include/kindyn/kinematics.hpp
# 		include/kindyn/cable.hpp)
# add_dependencies(kindyn roboy_simulation_msgs_generate_messages_cpp roboy_simulation_msgs_msgs_generate_messages_cpp)
# target_link_libraries(kindyn ${catkin_LIBRARIES} ${iDynTree_LIBRARIES} ${qpOASES_LIBRARIES} CardsflowHardwareInterface
# 		CableLengthController TorquePositionController ForcePositionController ${OpenMP_CXX_libraries})

# # vrpuppet library
# add_library(vrpuppet SHARED
# 		src/vrpuppet.cpp
# 		src/EigenExtension.cpp
# 		include/kindyn/EigenExtension.hpp
# 		include/kindyn/vrpuppet.hpp
# 		include/kindyn/cable.hpp)
# add_dependencies(vrpuppet roboy_simulation_msgs_generate_messages_cpp roboy_simulation_msgs_msgs_generate_messages_cpp)
# target_link_libraries(vrpuppet ${catkin_LIBRARIES} ${iDynTree_LIBRARIES} ${qpOASES_LIBRARIES} CardsflowHardwareInterface
# 		CableLengthController TorquePositionController ForcePositionController ${OpenMP_CXX_libraries})

# # testing idyntree
# add_executable( test_idyntree src/utilities/test_idyntree.cpp)
# target_link_libraries(test_idyntree ${iDynTree_LIBRARIES} ${catkin_LIBRARIES})

# # upper_body
# add_executable( upper_body src/robots/upper_body.cpp)
# target_link_libraries(upper_body ${iDynTree_LIBRARIES} ${catkin_LIBRARIES} vrpuppet kindyn)

# #==============================================================================
# # Installation
# #==============================================================================
# install(TARGETS
# 		kindyn
# 		CardsflowHardwareInterface
#         LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# )

# install(TARGETS
# 	    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(FILES
#         package.xml
# 		CableLengthController_plugin.xml
# 		TorquePositionController_plugin.xml
# 		ForcePositionController_plugin.xml
#         DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

# install(DIRECTORY
# 		include/kindyn/
# 		include/kindyn/controller
# 		DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# 		FILES_MATCHING PATTERN "*.hpp"
# 		)

ament_package()
