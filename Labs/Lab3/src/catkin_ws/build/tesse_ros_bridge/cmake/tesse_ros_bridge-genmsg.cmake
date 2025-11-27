# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "tesse_ros_bridge: 1 messages, 3 services")

set(MSG_I_FLAGS "-Itesse_ros_bridge:/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/src/tesse-ros-bridge/ROS/msg;-Itf:/opt/anaconda3/envs/ros_env/share/tf/cmake/../msg;-Istd_msgs:/opt/anaconda3/envs/ros_env/share/std_msgs/cmake/../msg;-Isensor_msgs:/opt/anaconda3/envs/ros_env/share/sensor_msgs/cmake/../msg;-Inav_msgs:/opt/anaconda3/envs/ros_env/share/nav_msgs/cmake/../msg;-Imav_msgs:/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/src/mav_comm/mav_msgs/msg;-Iackermann_msgs:/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/src/ackermann_msgs/msg;-Igeometry_msgs:/opt/anaconda3/envs/ros_env/share/geometry_msgs/cmake/../msg;-Iactionlib_msgs:/opt/anaconda3/envs/ros_env/share/actionlib_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(tesse_ros_bridge_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/src/tesse-ros-bridge/ROS/msg/CollisionStats.msg" NAME_WE)
add_custom_target(_tesse_ros_bridge_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tesse_ros_bridge" "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/src/tesse-ros-bridge/ROS/msg/CollisionStats.msg" "std_msgs/Header"
)

get_filename_component(_filename "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/src/tesse-ros-bridge/ROS/srv/ObjectSpawnRequestService.srv" NAME_WE)
add_custom_target(_tesse_ros_bridge_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tesse_ros_bridge" "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/src/tesse-ros-bridge/ROS/srv/ObjectSpawnRequestService.srv" "geometry_msgs/Pose:geometry_msgs/Quaternion:geometry_msgs/Point"
)

get_filename_component(_filename "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/src/tesse-ros-bridge/ROS/srv/RepositionRequestService.srv" NAME_WE)
add_custom_target(_tesse_ros_bridge_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tesse_ros_bridge" "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/src/tesse-ros-bridge/ROS/srv/RepositionRequestService.srv" "geometry_msgs/Pose:geometry_msgs/Quaternion:geometry_msgs/Point"
)

get_filename_component(_filename "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/src/tesse-ros-bridge/ROS/srv/SceneRequestService.srv" NAME_WE)
add_custom_target(_tesse_ros_bridge_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tesse_ros_bridge" "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/src/tesse-ros-bridge/ROS/srv/SceneRequestService.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(tesse_ros_bridge
  "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/src/tesse-ros-bridge/ROS/msg/CollisionStats.msg"
  "${MSG_I_FLAGS}"
  "/opt/anaconda3/envs/ros_env/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tesse_ros_bridge
)

### Generating Services
_generate_srv_cpp(tesse_ros_bridge
  "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/src/tesse-ros-bridge/ROS/srv/ObjectSpawnRequestService.srv"
  "${MSG_I_FLAGS}"
  "/opt/anaconda3/envs/ros_env/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/anaconda3/envs/ros_env/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/anaconda3/envs/ros_env/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tesse_ros_bridge
)
_generate_srv_cpp(tesse_ros_bridge
  "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/src/tesse-ros-bridge/ROS/srv/RepositionRequestService.srv"
  "${MSG_I_FLAGS}"
  "/opt/anaconda3/envs/ros_env/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/anaconda3/envs/ros_env/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/anaconda3/envs/ros_env/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tesse_ros_bridge
)
_generate_srv_cpp(tesse_ros_bridge
  "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/src/tesse-ros-bridge/ROS/srv/SceneRequestService.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tesse_ros_bridge
)

### Generating Module File
_generate_module_cpp(tesse_ros_bridge
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tesse_ros_bridge
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(tesse_ros_bridge_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(tesse_ros_bridge_generate_messages tesse_ros_bridge_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/src/tesse-ros-bridge/ROS/msg/CollisionStats.msg" NAME_WE)
add_dependencies(tesse_ros_bridge_generate_messages_cpp _tesse_ros_bridge_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/src/tesse-ros-bridge/ROS/srv/ObjectSpawnRequestService.srv" NAME_WE)
add_dependencies(tesse_ros_bridge_generate_messages_cpp _tesse_ros_bridge_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/src/tesse-ros-bridge/ROS/srv/RepositionRequestService.srv" NAME_WE)
add_dependencies(tesse_ros_bridge_generate_messages_cpp _tesse_ros_bridge_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/src/tesse-ros-bridge/ROS/srv/SceneRequestService.srv" NAME_WE)
add_dependencies(tesse_ros_bridge_generate_messages_cpp _tesse_ros_bridge_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tesse_ros_bridge_gencpp)
add_dependencies(tesse_ros_bridge_gencpp tesse_ros_bridge_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tesse_ros_bridge_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(tesse_ros_bridge
  "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/src/tesse-ros-bridge/ROS/msg/CollisionStats.msg"
  "${MSG_I_FLAGS}"
  "/opt/anaconda3/envs/ros_env/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tesse_ros_bridge
)

### Generating Services
_generate_srv_eus(tesse_ros_bridge
  "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/src/tesse-ros-bridge/ROS/srv/ObjectSpawnRequestService.srv"
  "${MSG_I_FLAGS}"
  "/opt/anaconda3/envs/ros_env/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/anaconda3/envs/ros_env/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/anaconda3/envs/ros_env/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tesse_ros_bridge
)
_generate_srv_eus(tesse_ros_bridge
  "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/src/tesse-ros-bridge/ROS/srv/RepositionRequestService.srv"
  "${MSG_I_FLAGS}"
  "/opt/anaconda3/envs/ros_env/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/anaconda3/envs/ros_env/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/anaconda3/envs/ros_env/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tesse_ros_bridge
)
_generate_srv_eus(tesse_ros_bridge
  "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/src/tesse-ros-bridge/ROS/srv/SceneRequestService.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tesse_ros_bridge
)

### Generating Module File
_generate_module_eus(tesse_ros_bridge
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tesse_ros_bridge
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(tesse_ros_bridge_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(tesse_ros_bridge_generate_messages tesse_ros_bridge_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/src/tesse-ros-bridge/ROS/msg/CollisionStats.msg" NAME_WE)
add_dependencies(tesse_ros_bridge_generate_messages_eus _tesse_ros_bridge_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/src/tesse-ros-bridge/ROS/srv/ObjectSpawnRequestService.srv" NAME_WE)
add_dependencies(tesse_ros_bridge_generate_messages_eus _tesse_ros_bridge_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/src/tesse-ros-bridge/ROS/srv/RepositionRequestService.srv" NAME_WE)
add_dependencies(tesse_ros_bridge_generate_messages_eus _tesse_ros_bridge_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/src/tesse-ros-bridge/ROS/srv/SceneRequestService.srv" NAME_WE)
add_dependencies(tesse_ros_bridge_generate_messages_eus _tesse_ros_bridge_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tesse_ros_bridge_geneus)
add_dependencies(tesse_ros_bridge_geneus tesse_ros_bridge_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tesse_ros_bridge_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(tesse_ros_bridge
  "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/src/tesse-ros-bridge/ROS/msg/CollisionStats.msg"
  "${MSG_I_FLAGS}"
  "/opt/anaconda3/envs/ros_env/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tesse_ros_bridge
)

### Generating Services
_generate_srv_lisp(tesse_ros_bridge
  "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/src/tesse-ros-bridge/ROS/srv/ObjectSpawnRequestService.srv"
  "${MSG_I_FLAGS}"
  "/opt/anaconda3/envs/ros_env/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/anaconda3/envs/ros_env/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/anaconda3/envs/ros_env/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tesse_ros_bridge
)
_generate_srv_lisp(tesse_ros_bridge
  "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/src/tesse-ros-bridge/ROS/srv/RepositionRequestService.srv"
  "${MSG_I_FLAGS}"
  "/opt/anaconda3/envs/ros_env/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/anaconda3/envs/ros_env/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/anaconda3/envs/ros_env/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tesse_ros_bridge
)
_generate_srv_lisp(tesse_ros_bridge
  "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/src/tesse-ros-bridge/ROS/srv/SceneRequestService.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tesse_ros_bridge
)

### Generating Module File
_generate_module_lisp(tesse_ros_bridge
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tesse_ros_bridge
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(tesse_ros_bridge_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(tesse_ros_bridge_generate_messages tesse_ros_bridge_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/src/tesse-ros-bridge/ROS/msg/CollisionStats.msg" NAME_WE)
add_dependencies(tesse_ros_bridge_generate_messages_lisp _tesse_ros_bridge_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/src/tesse-ros-bridge/ROS/srv/ObjectSpawnRequestService.srv" NAME_WE)
add_dependencies(tesse_ros_bridge_generate_messages_lisp _tesse_ros_bridge_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/src/tesse-ros-bridge/ROS/srv/RepositionRequestService.srv" NAME_WE)
add_dependencies(tesse_ros_bridge_generate_messages_lisp _tesse_ros_bridge_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/src/tesse-ros-bridge/ROS/srv/SceneRequestService.srv" NAME_WE)
add_dependencies(tesse_ros_bridge_generate_messages_lisp _tesse_ros_bridge_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tesse_ros_bridge_genlisp)
add_dependencies(tesse_ros_bridge_genlisp tesse_ros_bridge_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tesse_ros_bridge_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(tesse_ros_bridge
  "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/src/tesse-ros-bridge/ROS/msg/CollisionStats.msg"
  "${MSG_I_FLAGS}"
  "/opt/anaconda3/envs/ros_env/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tesse_ros_bridge
)

### Generating Services
_generate_srv_nodejs(tesse_ros_bridge
  "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/src/tesse-ros-bridge/ROS/srv/ObjectSpawnRequestService.srv"
  "${MSG_I_FLAGS}"
  "/opt/anaconda3/envs/ros_env/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/anaconda3/envs/ros_env/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/anaconda3/envs/ros_env/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tesse_ros_bridge
)
_generate_srv_nodejs(tesse_ros_bridge
  "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/src/tesse-ros-bridge/ROS/srv/RepositionRequestService.srv"
  "${MSG_I_FLAGS}"
  "/opt/anaconda3/envs/ros_env/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/anaconda3/envs/ros_env/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/anaconda3/envs/ros_env/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tesse_ros_bridge
)
_generate_srv_nodejs(tesse_ros_bridge
  "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/src/tesse-ros-bridge/ROS/srv/SceneRequestService.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tesse_ros_bridge
)

### Generating Module File
_generate_module_nodejs(tesse_ros_bridge
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tesse_ros_bridge
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(tesse_ros_bridge_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(tesse_ros_bridge_generate_messages tesse_ros_bridge_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/src/tesse-ros-bridge/ROS/msg/CollisionStats.msg" NAME_WE)
add_dependencies(tesse_ros_bridge_generate_messages_nodejs _tesse_ros_bridge_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/src/tesse-ros-bridge/ROS/srv/ObjectSpawnRequestService.srv" NAME_WE)
add_dependencies(tesse_ros_bridge_generate_messages_nodejs _tesse_ros_bridge_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/src/tesse-ros-bridge/ROS/srv/RepositionRequestService.srv" NAME_WE)
add_dependencies(tesse_ros_bridge_generate_messages_nodejs _tesse_ros_bridge_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/src/tesse-ros-bridge/ROS/srv/SceneRequestService.srv" NAME_WE)
add_dependencies(tesse_ros_bridge_generate_messages_nodejs _tesse_ros_bridge_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tesse_ros_bridge_gennodejs)
add_dependencies(tesse_ros_bridge_gennodejs tesse_ros_bridge_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tesse_ros_bridge_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(tesse_ros_bridge
  "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/src/tesse-ros-bridge/ROS/msg/CollisionStats.msg"
  "${MSG_I_FLAGS}"
  "/opt/anaconda3/envs/ros_env/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tesse_ros_bridge
)

### Generating Services
_generate_srv_py(tesse_ros_bridge
  "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/src/tesse-ros-bridge/ROS/srv/ObjectSpawnRequestService.srv"
  "${MSG_I_FLAGS}"
  "/opt/anaconda3/envs/ros_env/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/anaconda3/envs/ros_env/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/anaconda3/envs/ros_env/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tesse_ros_bridge
)
_generate_srv_py(tesse_ros_bridge
  "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/src/tesse-ros-bridge/ROS/srv/RepositionRequestService.srv"
  "${MSG_I_FLAGS}"
  "/opt/anaconda3/envs/ros_env/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/anaconda3/envs/ros_env/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/anaconda3/envs/ros_env/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tesse_ros_bridge
)
_generate_srv_py(tesse_ros_bridge
  "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/src/tesse-ros-bridge/ROS/srv/SceneRequestService.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tesse_ros_bridge
)

### Generating Module File
_generate_module_py(tesse_ros_bridge
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tesse_ros_bridge
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(tesse_ros_bridge_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(tesse_ros_bridge_generate_messages tesse_ros_bridge_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/src/tesse-ros-bridge/ROS/msg/CollisionStats.msg" NAME_WE)
add_dependencies(tesse_ros_bridge_generate_messages_py _tesse_ros_bridge_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/src/tesse-ros-bridge/ROS/srv/ObjectSpawnRequestService.srv" NAME_WE)
add_dependencies(tesse_ros_bridge_generate_messages_py _tesse_ros_bridge_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/src/tesse-ros-bridge/ROS/srv/RepositionRequestService.srv" NAME_WE)
add_dependencies(tesse_ros_bridge_generate_messages_py _tesse_ros_bridge_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/src/tesse-ros-bridge/ROS/srv/SceneRequestService.srv" NAME_WE)
add_dependencies(tesse_ros_bridge_generate_messages_py _tesse_ros_bridge_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tesse_ros_bridge_genpy)
add_dependencies(tesse_ros_bridge_genpy tesse_ros_bridge_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tesse_ros_bridge_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tesse_ros_bridge)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tesse_ros_bridge
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET tf_generate_messages_cpp)
  add_dependencies(tesse_ros_bridge_generate_messages_cpp tf_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(tesse_ros_bridge_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(tesse_ros_bridge_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()
if(TARGET nav_msgs_generate_messages_cpp)
  add_dependencies(tesse_ros_bridge_generate_messages_cpp nav_msgs_generate_messages_cpp)
endif()
if(TARGET mav_msgs_generate_messages_cpp)
  add_dependencies(tesse_ros_bridge_generate_messages_cpp mav_msgs_generate_messages_cpp)
endif()
if(TARGET ackermann_msgs_generate_messages_cpp)
  add_dependencies(tesse_ros_bridge_generate_messages_cpp ackermann_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tesse_ros_bridge)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tesse_ros_bridge
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET tf_generate_messages_eus)
  add_dependencies(tesse_ros_bridge_generate_messages_eus tf_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(tesse_ros_bridge_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(tesse_ros_bridge_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()
if(TARGET nav_msgs_generate_messages_eus)
  add_dependencies(tesse_ros_bridge_generate_messages_eus nav_msgs_generate_messages_eus)
endif()
if(TARGET mav_msgs_generate_messages_eus)
  add_dependencies(tesse_ros_bridge_generate_messages_eus mav_msgs_generate_messages_eus)
endif()
if(TARGET ackermann_msgs_generate_messages_eus)
  add_dependencies(tesse_ros_bridge_generate_messages_eus ackermann_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tesse_ros_bridge)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tesse_ros_bridge
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET tf_generate_messages_lisp)
  add_dependencies(tesse_ros_bridge_generate_messages_lisp tf_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(tesse_ros_bridge_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(tesse_ros_bridge_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()
if(TARGET nav_msgs_generate_messages_lisp)
  add_dependencies(tesse_ros_bridge_generate_messages_lisp nav_msgs_generate_messages_lisp)
endif()
if(TARGET mav_msgs_generate_messages_lisp)
  add_dependencies(tesse_ros_bridge_generate_messages_lisp mav_msgs_generate_messages_lisp)
endif()
if(TARGET ackermann_msgs_generate_messages_lisp)
  add_dependencies(tesse_ros_bridge_generate_messages_lisp ackermann_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tesse_ros_bridge)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tesse_ros_bridge
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET tf_generate_messages_nodejs)
  add_dependencies(tesse_ros_bridge_generate_messages_nodejs tf_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(tesse_ros_bridge_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(tesse_ros_bridge_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()
if(TARGET nav_msgs_generate_messages_nodejs)
  add_dependencies(tesse_ros_bridge_generate_messages_nodejs nav_msgs_generate_messages_nodejs)
endif()
if(TARGET mav_msgs_generate_messages_nodejs)
  add_dependencies(tesse_ros_bridge_generate_messages_nodejs mav_msgs_generate_messages_nodejs)
endif()
if(TARGET ackermann_msgs_generate_messages_nodejs)
  add_dependencies(tesse_ros_bridge_generate_messages_nodejs ackermann_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tesse_ros_bridge)
  install(CODE "execute_process(COMMAND \"/opt/anaconda3/envs/ros_env/bin/python3.9\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tesse_ros_bridge\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tesse_ros_bridge
    DESTINATION ${genpy_INSTALL_DIR}
    # skip all init files
    PATTERN "__init__.py" EXCLUDE
    PATTERN "__init__.pyc" EXCLUDE
  )
  # install init files which are not in the root folder of the generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tesse_ros_bridge
    DESTINATION ${genpy_INSTALL_DIR}
    FILES_MATCHING
    REGEX "${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tesse_ros_bridge/.+/__init__.pyc?$"
  )
endif()
if(TARGET tf_generate_messages_py)
  add_dependencies(tesse_ros_bridge_generate_messages_py tf_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(tesse_ros_bridge_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(tesse_ros_bridge_generate_messages_py sensor_msgs_generate_messages_py)
endif()
if(TARGET nav_msgs_generate_messages_py)
  add_dependencies(tesse_ros_bridge_generate_messages_py nav_msgs_generate_messages_py)
endif()
if(TARGET mav_msgs_generate_messages_py)
  add_dependencies(tesse_ros_bridge_generate_messages_py mav_msgs_generate_messages_py)
endif()
if(TARGET ackermann_msgs_generate_messages_py)
  add_dependencies(tesse_ros_bridge_generate_messages_py ackermann_msgs_generate_messages_py)
endif()
