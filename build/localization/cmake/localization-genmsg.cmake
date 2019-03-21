# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "localization: 9 messages, 0 services")

set(MSG_I_FLAGS "-Ilocalization:/home/lian/robot_ws/src/localization/msg;-Isensor_msgs:/opt/ros/kinetic/share/sensor_msgs/cmake/../msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg;-Itf:/opt/ros/kinetic/share/tf/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(localization_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/lian/robot_ws/src/localization/msg/LinesDetected.msg" NAME_WE)
add_custom_target(_localization_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "localization" "/home/lian/robot_ws/src/localization/msg/LinesDetected.msg" ""
)

get_filename_component(_filename "/home/lian/robot_ws/src/localization/msg/WorldObjects.msg" NAME_WE)
add_custom_target(_localization_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "localization" "/home/lian/robot_ws/src/localization/msg/WorldObjects.msg" "localization/GoalpostsDetected:std_msgs/Header:localization/LinesDetected:localization/ObjectsDetected:localization/ObstaclesDetected:geometry_msgs/Pose2D"
)

get_filename_component(_filename "/home/lian/robot_ws/src/localization/msg/OutputData.msg" NAME_WE)
add_custom_target(_localization_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "localization" "/home/lian/robot_ws/src/localization/msg/OutputData.msg" "geometry_msgs/Pose2D:std_msgs/Header:geometry_msgs/Point"
)

get_filename_component(_filename "/home/lian/robot_ws/src/localization/msg/MeanPoseConfStamped.msg" NAME_WE)
add_custom_target(_localization_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "localization" "/home/lian/robot_ws/src/localization/msg/MeanPoseConfStamped.msg" "geometry_msgs/Pose2D:std_msgs/Header"
)

get_filename_component(_filename "/home/lian/robot_ws/src/localization/msg/Particle.msg" NAME_WE)
add_custom_target(_localization_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "localization" "/home/lian/robot_ws/src/localization/msg/Particle.msg" "geometry_msgs/Pose2D"
)

get_filename_component(_filename "/home/lian/robot_ws/src/localization/msg/ParticleSet.msg" NAME_WE)
add_custom_target(_localization_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "localization" "/home/lian/robot_ws/src/localization/msg/ParticleSet.msg" "localization/Particle:geometry_msgs/Pose2D:std_msgs/Header"
)

get_filename_component(_filename "/home/lian/robot_ws/src/localization/msg/ObstaclesDetected.msg" NAME_WE)
add_custom_target(_localization_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "localization" "/home/lian/robot_ws/src/localization/msg/ObstaclesDetected.msg" "geometry_msgs/Pose2D"
)

get_filename_component(_filename "/home/lian/robot_ws/src/localization/msg/GoalpostsDetected.msg" NAME_WE)
add_custom_target(_localization_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "localization" "/home/lian/robot_ws/src/localization/msg/GoalpostsDetected.msg" "geometry_msgs/Pose2D"
)

get_filename_component(_filename "/home/lian/robot_ws/src/localization/msg/ObjectsDetected.msg" NAME_WE)
add_custom_target(_localization_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "localization" "/home/lian/robot_ws/src/localization/msg/ObjectsDetected.msg" "geometry_msgs/Pose2D"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(localization
  "/home/lian/robot_ws/src/localization/msg/LinesDetected.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/localization
)
_generate_msg_cpp(localization
  "/home/lian/robot_ws/src/localization/msg/WorldObjects.msg"
  "${MSG_I_FLAGS}"
  "/home/lian/robot_ws/src/localization/msg/GoalpostsDetected.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/lian/robot_ws/src/localization/msg/LinesDetected.msg;/home/lian/robot_ws/src/localization/msg/ObjectsDetected.msg;/home/lian/robot_ws/src/localization/msg/ObstaclesDetected.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/localization
)
_generate_msg_cpp(localization
  "/home/lian/robot_ws/src/localization/msg/OutputData.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/localization
)
_generate_msg_cpp(localization
  "/home/lian/robot_ws/src/localization/msg/MeanPoseConfStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/localization
)
_generate_msg_cpp(localization
  "/home/lian/robot_ws/src/localization/msg/Particle.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/localization
)
_generate_msg_cpp(localization
  "/home/lian/robot_ws/src/localization/msg/GoalpostsDetected.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/localization
)
_generate_msg_cpp(localization
  "/home/lian/robot_ws/src/localization/msg/ObstaclesDetected.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/localization
)
_generate_msg_cpp(localization
  "/home/lian/robot_ws/src/localization/msg/ObjectsDetected.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/localization
)
_generate_msg_cpp(localization
  "/home/lian/robot_ws/src/localization/msg/ParticleSet.msg"
  "${MSG_I_FLAGS}"
  "/home/lian/robot_ws/src/localization/msg/Particle.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/localization
)

### Generating Services

### Generating Module File
_generate_module_cpp(localization
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/localization
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(localization_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(localization_generate_messages localization_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lian/robot_ws/src/localization/msg/LinesDetected.msg" NAME_WE)
add_dependencies(localization_generate_messages_cpp _localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/localization/msg/WorldObjects.msg" NAME_WE)
add_dependencies(localization_generate_messages_cpp _localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/localization/msg/OutputData.msg" NAME_WE)
add_dependencies(localization_generate_messages_cpp _localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/localization/msg/MeanPoseConfStamped.msg" NAME_WE)
add_dependencies(localization_generate_messages_cpp _localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/localization/msg/Particle.msg" NAME_WE)
add_dependencies(localization_generate_messages_cpp _localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/localization/msg/ParticleSet.msg" NAME_WE)
add_dependencies(localization_generate_messages_cpp _localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/localization/msg/ObstaclesDetected.msg" NAME_WE)
add_dependencies(localization_generate_messages_cpp _localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/localization/msg/GoalpostsDetected.msg" NAME_WE)
add_dependencies(localization_generate_messages_cpp _localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/localization/msg/ObjectsDetected.msg" NAME_WE)
add_dependencies(localization_generate_messages_cpp _localization_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(localization_gencpp)
add_dependencies(localization_gencpp localization_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS localization_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(localization
  "/home/lian/robot_ws/src/localization/msg/LinesDetected.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/localization
)
_generate_msg_eus(localization
  "/home/lian/robot_ws/src/localization/msg/WorldObjects.msg"
  "${MSG_I_FLAGS}"
  "/home/lian/robot_ws/src/localization/msg/GoalpostsDetected.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/lian/robot_ws/src/localization/msg/LinesDetected.msg;/home/lian/robot_ws/src/localization/msg/ObjectsDetected.msg;/home/lian/robot_ws/src/localization/msg/ObstaclesDetected.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/localization
)
_generate_msg_eus(localization
  "/home/lian/robot_ws/src/localization/msg/OutputData.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/localization
)
_generate_msg_eus(localization
  "/home/lian/robot_ws/src/localization/msg/MeanPoseConfStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/localization
)
_generate_msg_eus(localization
  "/home/lian/robot_ws/src/localization/msg/Particle.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/localization
)
_generate_msg_eus(localization
  "/home/lian/robot_ws/src/localization/msg/GoalpostsDetected.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/localization
)
_generate_msg_eus(localization
  "/home/lian/robot_ws/src/localization/msg/ObstaclesDetected.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/localization
)
_generate_msg_eus(localization
  "/home/lian/robot_ws/src/localization/msg/ObjectsDetected.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/localization
)
_generate_msg_eus(localization
  "/home/lian/robot_ws/src/localization/msg/ParticleSet.msg"
  "${MSG_I_FLAGS}"
  "/home/lian/robot_ws/src/localization/msg/Particle.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/localization
)

### Generating Services

### Generating Module File
_generate_module_eus(localization
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/localization
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(localization_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(localization_generate_messages localization_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lian/robot_ws/src/localization/msg/LinesDetected.msg" NAME_WE)
add_dependencies(localization_generate_messages_eus _localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/localization/msg/WorldObjects.msg" NAME_WE)
add_dependencies(localization_generate_messages_eus _localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/localization/msg/OutputData.msg" NAME_WE)
add_dependencies(localization_generate_messages_eus _localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/localization/msg/MeanPoseConfStamped.msg" NAME_WE)
add_dependencies(localization_generate_messages_eus _localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/localization/msg/Particle.msg" NAME_WE)
add_dependencies(localization_generate_messages_eus _localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/localization/msg/ParticleSet.msg" NAME_WE)
add_dependencies(localization_generate_messages_eus _localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/localization/msg/ObstaclesDetected.msg" NAME_WE)
add_dependencies(localization_generate_messages_eus _localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/localization/msg/GoalpostsDetected.msg" NAME_WE)
add_dependencies(localization_generate_messages_eus _localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/localization/msg/ObjectsDetected.msg" NAME_WE)
add_dependencies(localization_generate_messages_eus _localization_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(localization_geneus)
add_dependencies(localization_geneus localization_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS localization_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(localization
  "/home/lian/robot_ws/src/localization/msg/LinesDetected.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/localization
)
_generate_msg_lisp(localization
  "/home/lian/robot_ws/src/localization/msg/WorldObjects.msg"
  "${MSG_I_FLAGS}"
  "/home/lian/robot_ws/src/localization/msg/GoalpostsDetected.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/lian/robot_ws/src/localization/msg/LinesDetected.msg;/home/lian/robot_ws/src/localization/msg/ObjectsDetected.msg;/home/lian/robot_ws/src/localization/msg/ObstaclesDetected.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/localization
)
_generate_msg_lisp(localization
  "/home/lian/robot_ws/src/localization/msg/OutputData.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/localization
)
_generate_msg_lisp(localization
  "/home/lian/robot_ws/src/localization/msg/MeanPoseConfStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/localization
)
_generate_msg_lisp(localization
  "/home/lian/robot_ws/src/localization/msg/Particle.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/localization
)
_generate_msg_lisp(localization
  "/home/lian/robot_ws/src/localization/msg/GoalpostsDetected.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/localization
)
_generate_msg_lisp(localization
  "/home/lian/robot_ws/src/localization/msg/ObstaclesDetected.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/localization
)
_generate_msg_lisp(localization
  "/home/lian/robot_ws/src/localization/msg/ObjectsDetected.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/localization
)
_generate_msg_lisp(localization
  "/home/lian/robot_ws/src/localization/msg/ParticleSet.msg"
  "${MSG_I_FLAGS}"
  "/home/lian/robot_ws/src/localization/msg/Particle.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/localization
)

### Generating Services

### Generating Module File
_generate_module_lisp(localization
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/localization
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(localization_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(localization_generate_messages localization_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lian/robot_ws/src/localization/msg/LinesDetected.msg" NAME_WE)
add_dependencies(localization_generate_messages_lisp _localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/localization/msg/WorldObjects.msg" NAME_WE)
add_dependencies(localization_generate_messages_lisp _localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/localization/msg/OutputData.msg" NAME_WE)
add_dependencies(localization_generate_messages_lisp _localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/localization/msg/MeanPoseConfStamped.msg" NAME_WE)
add_dependencies(localization_generate_messages_lisp _localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/localization/msg/Particle.msg" NAME_WE)
add_dependencies(localization_generate_messages_lisp _localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/localization/msg/ParticleSet.msg" NAME_WE)
add_dependencies(localization_generate_messages_lisp _localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/localization/msg/ObstaclesDetected.msg" NAME_WE)
add_dependencies(localization_generate_messages_lisp _localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/localization/msg/GoalpostsDetected.msg" NAME_WE)
add_dependencies(localization_generate_messages_lisp _localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/localization/msg/ObjectsDetected.msg" NAME_WE)
add_dependencies(localization_generate_messages_lisp _localization_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(localization_genlisp)
add_dependencies(localization_genlisp localization_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS localization_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(localization
  "/home/lian/robot_ws/src/localization/msg/LinesDetected.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/localization
)
_generate_msg_nodejs(localization
  "/home/lian/robot_ws/src/localization/msg/WorldObjects.msg"
  "${MSG_I_FLAGS}"
  "/home/lian/robot_ws/src/localization/msg/GoalpostsDetected.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/lian/robot_ws/src/localization/msg/LinesDetected.msg;/home/lian/robot_ws/src/localization/msg/ObjectsDetected.msg;/home/lian/robot_ws/src/localization/msg/ObstaclesDetected.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/localization
)
_generate_msg_nodejs(localization
  "/home/lian/robot_ws/src/localization/msg/OutputData.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/localization
)
_generate_msg_nodejs(localization
  "/home/lian/robot_ws/src/localization/msg/MeanPoseConfStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/localization
)
_generate_msg_nodejs(localization
  "/home/lian/robot_ws/src/localization/msg/Particle.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/localization
)
_generate_msg_nodejs(localization
  "/home/lian/robot_ws/src/localization/msg/GoalpostsDetected.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/localization
)
_generate_msg_nodejs(localization
  "/home/lian/robot_ws/src/localization/msg/ObstaclesDetected.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/localization
)
_generate_msg_nodejs(localization
  "/home/lian/robot_ws/src/localization/msg/ObjectsDetected.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/localization
)
_generate_msg_nodejs(localization
  "/home/lian/robot_ws/src/localization/msg/ParticleSet.msg"
  "${MSG_I_FLAGS}"
  "/home/lian/robot_ws/src/localization/msg/Particle.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/localization
)

### Generating Services

### Generating Module File
_generate_module_nodejs(localization
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/localization
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(localization_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(localization_generate_messages localization_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lian/robot_ws/src/localization/msg/LinesDetected.msg" NAME_WE)
add_dependencies(localization_generate_messages_nodejs _localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/localization/msg/WorldObjects.msg" NAME_WE)
add_dependencies(localization_generate_messages_nodejs _localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/localization/msg/OutputData.msg" NAME_WE)
add_dependencies(localization_generate_messages_nodejs _localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/localization/msg/MeanPoseConfStamped.msg" NAME_WE)
add_dependencies(localization_generate_messages_nodejs _localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/localization/msg/Particle.msg" NAME_WE)
add_dependencies(localization_generate_messages_nodejs _localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/localization/msg/ParticleSet.msg" NAME_WE)
add_dependencies(localization_generate_messages_nodejs _localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/localization/msg/ObstaclesDetected.msg" NAME_WE)
add_dependencies(localization_generate_messages_nodejs _localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/localization/msg/GoalpostsDetected.msg" NAME_WE)
add_dependencies(localization_generate_messages_nodejs _localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/localization/msg/ObjectsDetected.msg" NAME_WE)
add_dependencies(localization_generate_messages_nodejs _localization_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(localization_gennodejs)
add_dependencies(localization_gennodejs localization_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS localization_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(localization
  "/home/lian/robot_ws/src/localization/msg/LinesDetected.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/localization
)
_generate_msg_py(localization
  "/home/lian/robot_ws/src/localization/msg/WorldObjects.msg"
  "${MSG_I_FLAGS}"
  "/home/lian/robot_ws/src/localization/msg/GoalpostsDetected.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/lian/robot_ws/src/localization/msg/LinesDetected.msg;/home/lian/robot_ws/src/localization/msg/ObjectsDetected.msg;/home/lian/robot_ws/src/localization/msg/ObstaclesDetected.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/localization
)
_generate_msg_py(localization
  "/home/lian/robot_ws/src/localization/msg/OutputData.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/localization
)
_generate_msg_py(localization
  "/home/lian/robot_ws/src/localization/msg/MeanPoseConfStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/localization
)
_generate_msg_py(localization
  "/home/lian/robot_ws/src/localization/msg/Particle.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/localization
)
_generate_msg_py(localization
  "/home/lian/robot_ws/src/localization/msg/GoalpostsDetected.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/localization
)
_generate_msg_py(localization
  "/home/lian/robot_ws/src/localization/msg/ObstaclesDetected.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/localization
)
_generate_msg_py(localization
  "/home/lian/robot_ws/src/localization/msg/ObjectsDetected.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/localization
)
_generate_msg_py(localization
  "/home/lian/robot_ws/src/localization/msg/ParticleSet.msg"
  "${MSG_I_FLAGS}"
  "/home/lian/robot_ws/src/localization/msg/Particle.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/localization
)

### Generating Services

### Generating Module File
_generate_module_py(localization
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/localization
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(localization_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(localization_generate_messages localization_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lian/robot_ws/src/localization/msg/LinesDetected.msg" NAME_WE)
add_dependencies(localization_generate_messages_py _localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/localization/msg/WorldObjects.msg" NAME_WE)
add_dependencies(localization_generate_messages_py _localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/localization/msg/OutputData.msg" NAME_WE)
add_dependencies(localization_generate_messages_py _localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/localization/msg/MeanPoseConfStamped.msg" NAME_WE)
add_dependencies(localization_generate_messages_py _localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/localization/msg/Particle.msg" NAME_WE)
add_dependencies(localization_generate_messages_py _localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/localization/msg/ParticleSet.msg" NAME_WE)
add_dependencies(localization_generate_messages_py _localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/localization/msg/ObstaclesDetected.msg" NAME_WE)
add_dependencies(localization_generate_messages_py _localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/localization/msg/GoalpostsDetected.msg" NAME_WE)
add_dependencies(localization_generate_messages_py _localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/localization/msg/ObjectsDetected.msg" NAME_WE)
add_dependencies(localization_generate_messages_py _localization_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(localization_genpy)
add_dependencies(localization_genpy localization_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS localization_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/localization)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/localization
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(localization_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(localization_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(localization_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET tf_generate_messages_cpp)
  add_dependencies(localization_generate_messages_cpp tf_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/localization)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/localization
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(localization_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(localization_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(localization_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET tf_generate_messages_eus)
  add_dependencies(localization_generate_messages_eus tf_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/localization)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/localization
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(localization_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(localization_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(localization_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET tf_generate_messages_lisp)
  add_dependencies(localization_generate_messages_lisp tf_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/localization)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/localization
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(localization_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(localization_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(localization_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET tf_generate_messages_nodejs)
  add_dependencies(localization_generate_messages_nodejs tf_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/localization)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/localization\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/localization
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(localization_generate_messages_py sensor_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(localization_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(localization_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET tf_generate_messages_py)
  add_dependencies(localization_generate_messages_py tf_generate_messages_py)
endif()
