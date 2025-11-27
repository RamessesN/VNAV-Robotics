# Install script for directory: /Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/src/mav_comm/mav_planning_msgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/opt/anaconda3/envs/ros_env/bin/llvm-objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
        file(MAKE_DIRECTORY "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
      endif()
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin")
        file(WRITE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin" "")
      endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/install/_setup_util.py")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/install" TYPE PROGRAM FILES "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/build/mav_planning_msgs/catkin_generated/installspace/_setup_util.py")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/install/env.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/install" TYPE PROGRAM FILES "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/build/mav_planning_msgs/catkin_generated/installspace/env.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/install/setup.bash;/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/install/local_setup.bash")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/install" TYPE FILE FILES
    "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/build/mav_planning_msgs/catkin_generated/installspace/setup.bash"
    "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/build/mav_planning_msgs/catkin_generated/installspace/local_setup.bash"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/install/setup.sh;/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/install/local_setup.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/install" TYPE FILE FILES
    "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/build/mav_planning_msgs/catkin_generated/installspace/setup.sh"
    "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/build/mav_planning_msgs/catkin_generated/installspace/local_setup.sh"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/install/setup.zsh;/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/install/local_setup.zsh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/install" TYPE FILE FILES
    "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/build/mav_planning_msgs/catkin_generated/installspace/setup.zsh"
    "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/build/mav_planning_msgs/catkin_generated/installspace/local_setup.zsh"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/install/.rosinstall")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/install" TYPE FILE FILES "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/build/mav_planning_msgs/catkin_generated/installspace/.rosinstall")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mav_planning_msgs/msg" TYPE FILE FILES
    "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/src/mav_comm/mav_planning_msgs/msg/Point2D.msg"
    "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/src/mav_comm/mav_planning_msgs/msg/PointCloudWithPose.msg"
    "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/src/mav_comm/mav_planning_msgs/msg/Polygon2D.msg"
    "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/src/mav_comm/mav_planning_msgs/msg/PolygonWithHoles.msg"
    "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/src/mav_comm/mav_planning_msgs/msg/PolygonWithHolesStamped.msg"
    "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/src/mav_comm/mav_planning_msgs/msg/PolynomialSegment.msg"
    "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/src/mav_comm/mav_planning_msgs/msg/PolynomialTrajectory.msg"
    "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/src/mav_comm/mav_planning_msgs/msg/PolynomialSegment4D.msg"
    "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/src/mav_comm/mav_planning_msgs/msg/PolynomialTrajectory4D.msg"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mav_planning_msgs/srv" TYPE FILE FILES
    "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/src/mav_comm/mav_planning_msgs/srv/PlannerService.srv"
    "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/src/mav_comm/mav_planning_msgs/srv/PolygonService.srv"
    "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/src/mav_comm/mav_planning_msgs/srv/ChangeNameService.srv"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mav_planning_msgs/cmake" TYPE FILE FILES "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/build/mav_planning_msgs/catkin_generated/installspace/mav_planning_msgs-msg-paths.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/devel/.private/mav_planning_msgs/include/mav_planning_msgs")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/devel/.private/mav_planning_msgs/share/roseus/ros/mav_planning_msgs")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/devel/.private/mav_planning_msgs/share/common-lisp/ros/mav_planning_msgs")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/devel/.private/mav_planning_msgs/share/gennodejs/ros/mav_planning_msgs")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/opt/anaconda3/envs/ros_env/bin/python3.9" -m compileall "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/devel/.private/mav_planning_msgs/lib/python3.9/site-packages/mav_planning_msgs")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3.9/site-packages" TYPE DIRECTORY FILES "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/devel/.private/mav_planning_msgs/lib/python3.9/site-packages/mav_planning_msgs")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/build/mav_planning_msgs/catkin_generated/installspace/mav_planning_msgs.pc")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mav_planning_msgs/cmake" TYPE FILE FILES "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/build/mav_planning_msgs/catkin_generated/installspace/mav_planning_msgs-msg-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mav_planning_msgs/cmake" TYPE FILE FILES
    "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/build/mav_planning_msgs/catkin_generated/installspace/mav_planning_msgsConfig.cmake"
    "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/build/mav_planning_msgs/catkin_generated/installspace/mav_planning_msgsConfig-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mav_planning_msgs" TYPE FILE FILES "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/src/mav_comm/mav_planning_msgs/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/build/mav_planning_msgs/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
