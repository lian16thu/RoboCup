# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "decision: 9 messages, 3 services")

set(MSG_I_FLAGS "-Idecision:/home/lian/robot_ws/src/decision/msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(decision_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/lian/robot_ws/src/decision/msg/Ball.msg" NAME_WE)
add_custom_target(_decision_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "decision" "/home/lian/robot_ws/src/decision/msg/Ball.msg" ""
)

get_filename_component(_filename "/home/lian/robot_ws/src/decision/srv/head.srv" NAME_WE)
add_custom_target(_decision_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "decision" "/home/lian/robot_ws/src/decision/srv/head.srv" ""
)

get_filename_component(_filename "/home/lian/robot_ws/src/decision/msg/gameControl.msg" NAME_WE)
add_custom_target(_decision_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "decision" "/home/lian/robot_ws/src/decision/msg/gameControl.msg" ""
)

get_filename_component(_filename "/home/lian/robot_ws/src/decision/msg/GoalData.msg" NAME_WE)
add_custom_target(_decision_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "decision" "/home/lian/robot_ws/src/decision/msg/GoalData.msg" ""
)

get_filename_component(_filename "/home/lian/robot_ws/src/decision/srv/Pathplaning_the.srv" NAME_WE)
add_custom_target(_decision_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "decision" "/home/lian/robot_ws/src/decision/srv/Pathplaning_the.srv" ""
)

get_filename_component(_filename "/home/lian/robot_ws/src/decision/msg/gyro_euler.msg" NAME_WE)
add_custom_target(_decision_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "decision" "/home/lian/robot_ws/src/decision/msg/gyro_euler.msg" "geometry_msgs/Vector3"
)

get_filename_component(_filename "/home/lian/robot_ws/src/decision/msg/UDPReceived.msg" NAME_WE)
add_custom_target(_decision_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "decision" "/home/lian/robot_ws/src/decision/msg/UDPReceived.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/lian/robot_ws/src/decision/msg/Obstacle.msg" NAME_WE)
add_custom_target(_decision_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "decision" "/home/lian/robot_ws/src/decision/msg/Obstacle.msg" ""
)

get_filename_component(_filename "/home/lian/robot_ws/src/decision/msg/head_angle.msg" NAME_WE)
add_custom_target(_decision_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "decision" "/home/lian/robot_ws/src/decision/msg/head_angle.msg" ""
)

get_filename_component(_filename "/home/lian/robot_ws/src/decision/msg/OutputData.msg" NAME_WE)
add_custom_target(_decision_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "decision" "/home/lian/robot_ws/src/decision/msg/OutputData.msg" "geometry_msgs/Pose2D:std_msgs/Header:geometry_msgs/Point"
)

get_filename_component(_filename "/home/lian/robot_ws/src/decision/srv/walk.srv" NAME_WE)
add_custom_target(_decision_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "decision" "/home/lian/robot_ws/src/decision/srv/walk.srv" ""
)

get_filename_component(_filename "/home/lian/robot_ws/src/decision/msg/SerialReceived.msg" NAME_WE)
add_custom_target(_decision_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "decision" "/home/lian/robot_ws/src/decision/msg/SerialReceived.msg" "std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(decision
  "/home/lian/robot_ws/src/decision/msg/Ball.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/decision
)
_generate_msg_cpp(decision
  "/home/lian/robot_ws/src/decision/msg/gameControl.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/decision
)
_generate_msg_cpp(decision
  "/home/lian/robot_ws/src/decision/msg/GoalData.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/decision
)
_generate_msg_cpp(decision
  "/home/lian/robot_ws/src/decision/msg/UDPReceived.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/decision
)
_generate_msg_cpp(decision
  "/home/lian/robot_ws/src/decision/msg/gyro_euler.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/decision
)
_generate_msg_cpp(decision
  "/home/lian/robot_ws/src/decision/msg/Obstacle.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/decision
)
_generate_msg_cpp(decision
  "/home/lian/robot_ws/src/decision/msg/head_angle.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/decision
)
_generate_msg_cpp(decision
  "/home/lian/robot_ws/src/decision/msg/OutputData.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/decision
)
_generate_msg_cpp(decision
  "/home/lian/robot_ws/src/decision/msg/SerialReceived.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/decision
)

### Generating Services
_generate_srv_cpp(decision
  "/home/lian/robot_ws/src/decision/srv/head.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/decision
)
_generate_srv_cpp(decision
  "/home/lian/robot_ws/src/decision/srv/Pathplaning_the.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/decision
)
_generate_srv_cpp(decision
  "/home/lian/robot_ws/src/decision/srv/walk.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/decision
)

### Generating Module File
_generate_module_cpp(decision
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/decision
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(decision_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(decision_generate_messages decision_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lian/robot_ws/src/decision/msg/Ball.msg" NAME_WE)
add_dependencies(decision_generate_messages_cpp _decision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/decision/srv/head.srv" NAME_WE)
add_dependencies(decision_generate_messages_cpp _decision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/decision/msg/gameControl.msg" NAME_WE)
add_dependencies(decision_generate_messages_cpp _decision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/decision/msg/GoalData.msg" NAME_WE)
add_dependencies(decision_generate_messages_cpp _decision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/decision/srv/Pathplaning_the.srv" NAME_WE)
add_dependencies(decision_generate_messages_cpp _decision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/decision/msg/gyro_euler.msg" NAME_WE)
add_dependencies(decision_generate_messages_cpp _decision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/decision/msg/UDPReceived.msg" NAME_WE)
add_dependencies(decision_generate_messages_cpp _decision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/decision/msg/Obstacle.msg" NAME_WE)
add_dependencies(decision_generate_messages_cpp _decision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/decision/msg/head_angle.msg" NAME_WE)
add_dependencies(decision_generate_messages_cpp _decision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/decision/msg/OutputData.msg" NAME_WE)
add_dependencies(decision_generate_messages_cpp _decision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/decision/srv/walk.srv" NAME_WE)
add_dependencies(decision_generate_messages_cpp _decision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/decision/msg/SerialReceived.msg" NAME_WE)
add_dependencies(decision_generate_messages_cpp _decision_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(decision_gencpp)
add_dependencies(decision_gencpp decision_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS decision_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(decision
  "/home/lian/robot_ws/src/decision/msg/Ball.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/decision
)
_generate_msg_eus(decision
  "/home/lian/robot_ws/src/decision/msg/gameControl.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/decision
)
_generate_msg_eus(decision
  "/home/lian/robot_ws/src/decision/msg/GoalData.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/decision
)
_generate_msg_eus(decision
  "/home/lian/robot_ws/src/decision/msg/UDPReceived.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/decision
)
_generate_msg_eus(decision
  "/home/lian/robot_ws/src/decision/msg/gyro_euler.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/decision
)
_generate_msg_eus(decision
  "/home/lian/robot_ws/src/decision/msg/Obstacle.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/decision
)
_generate_msg_eus(decision
  "/home/lian/robot_ws/src/decision/msg/head_angle.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/decision
)
_generate_msg_eus(decision
  "/home/lian/robot_ws/src/decision/msg/OutputData.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/decision
)
_generate_msg_eus(decision
  "/home/lian/robot_ws/src/decision/msg/SerialReceived.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/decision
)

### Generating Services
_generate_srv_eus(decision
  "/home/lian/robot_ws/src/decision/srv/head.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/decision
)
_generate_srv_eus(decision
  "/home/lian/robot_ws/src/decision/srv/Pathplaning_the.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/decision
)
_generate_srv_eus(decision
  "/home/lian/robot_ws/src/decision/srv/walk.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/decision
)

### Generating Module File
_generate_module_eus(decision
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/decision
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(decision_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(decision_generate_messages decision_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lian/robot_ws/src/decision/msg/Ball.msg" NAME_WE)
add_dependencies(decision_generate_messages_eus _decision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/decision/srv/head.srv" NAME_WE)
add_dependencies(decision_generate_messages_eus _decision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/decision/msg/gameControl.msg" NAME_WE)
add_dependencies(decision_generate_messages_eus _decision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/decision/msg/GoalData.msg" NAME_WE)
add_dependencies(decision_generate_messages_eus _decision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/decision/srv/Pathplaning_the.srv" NAME_WE)
add_dependencies(decision_generate_messages_eus _decision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/decision/msg/gyro_euler.msg" NAME_WE)
add_dependencies(decision_generate_messages_eus _decision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/decision/msg/UDPReceived.msg" NAME_WE)
add_dependencies(decision_generate_messages_eus _decision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/decision/msg/Obstacle.msg" NAME_WE)
add_dependencies(decision_generate_messages_eus _decision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/decision/msg/head_angle.msg" NAME_WE)
add_dependencies(decision_generate_messages_eus _decision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/decision/msg/OutputData.msg" NAME_WE)
add_dependencies(decision_generate_messages_eus _decision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/decision/srv/walk.srv" NAME_WE)
add_dependencies(decision_generate_messages_eus _decision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/decision/msg/SerialReceived.msg" NAME_WE)
add_dependencies(decision_generate_messages_eus _decision_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(decision_geneus)
add_dependencies(decision_geneus decision_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS decision_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(decision
  "/home/lian/robot_ws/src/decision/msg/Ball.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/decision
)
_generate_msg_lisp(decision
  "/home/lian/robot_ws/src/decision/msg/gameControl.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/decision
)
_generate_msg_lisp(decision
  "/home/lian/robot_ws/src/decision/msg/GoalData.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/decision
)
_generate_msg_lisp(decision
  "/home/lian/robot_ws/src/decision/msg/UDPReceived.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/decision
)
_generate_msg_lisp(decision
  "/home/lian/robot_ws/src/decision/msg/gyro_euler.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/decision
)
_generate_msg_lisp(decision
  "/home/lian/robot_ws/src/decision/msg/Obstacle.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/decision
)
_generate_msg_lisp(decision
  "/home/lian/robot_ws/src/decision/msg/head_angle.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/decision
)
_generate_msg_lisp(decision
  "/home/lian/robot_ws/src/decision/msg/OutputData.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/decision
)
_generate_msg_lisp(decision
  "/home/lian/robot_ws/src/decision/msg/SerialReceived.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/decision
)

### Generating Services
_generate_srv_lisp(decision
  "/home/lian/robot_ws/src/decision/srv/head.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/decision
)
_generate_srv_lisp(decision
  "/home/lian/robot_ws/src/decision/srv/Pathplaning_the.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/decision
)
_generate_srv_lisp(decision
  "/home/lian/robot_ws/src/decision/srv/walk.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/decision
)

### Generating Module File
_generate_module_lisp(decision
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/decision
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(decision_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(decision_generate_messages decision_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lian/robot_ws/src/decision/msg/Ball.msg" NAME_WE)
add_dependencies(decision_generate_messages_lisp _decision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/decision/srv/head.srv" NAME_WE)
add_dependencies(decision_generate_messages_lisp _decision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/decision/msg/gameControl.msg" NAME_WE)
add_dependencies(decision_generate_messages_lisp _decision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/decision/msg/GoalData.msg" NAME_WE)
add_dependencies(decision_generate_messages_lisp _decision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/decision/srv/Pathplaning_the.srv" NAME_WE)
add_dependencies(decision_generate_messages_lisp _decision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/decision/msg/gyro_euler.msg" NAME_WE)
add_dependencies(decision_generate_messages_lisp _decision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/decision/msg/UDPReceived.msg" NAME_WE)
add_dependencies(decision_generate_messages_lisp _decision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/decision/msg/Obstacle.msg" NAME_WE)
add_dependencies(decision_generate_messages_lisp _decision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/decision/msg/head_angle.msg" NAME_WE)
add_dependencies(decision_generate_messages_lisp _decision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/decision/msg/OutputData.msg" NAME_WE)
add_dependencies(decision_generate_messages_lisp _decision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/decision/srv/walk.srv" NAME_WE)
add_dependencies(decision_generate_messages_lisp _decision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/decision/msg/SerialReceived.msg" NAME_WE)
add_dependencies(decision_generate_messages_lisp _decision_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(decision_genlisp)
add_dependencies(decision_genlisp decision_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS decision_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(decision
  "/home/lian/robot_ws/src/decision/msg/Ball.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/decision
)
_generate_msg_nodejs(decision
  "/home/lian/robot_ws/src/decision/msg/gameControl.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/decision
)
_generate_msg_nodejs(decision
  "/home/lian/robot_ws/src/decision/msg/GoalData.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/decision
)
_generate_msg_nodejs(decision
  "/home/lian/robot_ws/src/decision/msg/UDPReceived.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/decision
)
_generate_msg_nodejs(decision
  "/home/lian/robot_ws/src/decision/msg/gyro_euler.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/decision
)
_generate_msg_nodejs(decision
  "/home/lian/robot_ws/src/decision/msg/Obstacle.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/decision
)
_generate_msg_nodejs(decision
  "/home/lian/robot_ws/src/decision/msg/head_angle.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/decision
)
_generate_msg_nodejs(decision
  "/home/lian/robot_ws/src/decision/msg/OutputData.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/decision
)
_generate_msg_nodejs(decision
  "/home/lian/robot_ws/src/decision/msg/SerialReceived.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/decision
)

### Generating Services
_generate_srv_nodejs(decision
  "/home/lian/robot_ws/src/decision/srv/head.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/decision
)
_generate_srv_nodejs(decision
  "/home/lian/robot_ws/src/decision/srv/Pathplaning_the.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/decision
)
_generate_srv_nodejs(decision
  "/home/lian/robot_ws/src/decision/srv/walk.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/decision
)

### Generating Module File
_generate_module_nodejs(decision
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/decision
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(decision_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(decision_generate_messages decision_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lian/robot_ws/src/decision/msg/Ball.msg" NAME_WE)
add_dependencies(decision_generate_messages_nodejs _decision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/decision/srv/head.srv" NAME_WE)
add_dependencies(decision_generate_messages_nodejs _decision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/decision/msg/gameControl.msg" NAME_WE)
add_dependencies(decision_generate_messages_nodejs _decision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/decision/msg/GoalData.msg" NAME_WE)
add_dependencies(decision_generate_messages_nodejs _decision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/decision/srv/Pathplaning_the.srv" NAME_WE)
add_dependencies(decision_generate_messages_nodejs _decision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/decision/msg/gyro_euler.msg" NAME_WE)
add_dependencies(decision_generate_messages_nodejs _decision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/decision/msg/UDPReceived.msg" NAME_WE)
add_dependencies(decision_generate_messages_nodejs _decision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/decision/msg/Obstacle.msg" NAME_WE)
add_dependencies(decision_generate_messages_nodejs _decision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/decision/msg/head_angle.msg" NAME_WE)
add_dependencies(decision_generate_messages_nodejs _decision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/decision/msg/OutputData.msg" NAME_WE)
add_dependencies(decision_generate_messages_nodejs _decision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/decision/srv/walk.srv" NAME_WE)
add_dependencies(decision_generate_messages_nodejs _decision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/decision/msg/SerialReceived.msg" NAME_WE)
add_dependencies(decision_generate_messages_nodejs _decision_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(decision_gennodejs)
add_dependencies(decision_gennodejs decision_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS decision_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(decision
  "/home/lian/robot_ws/src/decision/msg/Ball.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/decision
)
_generate_msg_py(decision
  "/home/lian/robot_ws/src/decision/msg/gameControl.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/decision
)
_generate_msg_py(decision
  "/home/lian/robot_ws/src/decision/msg/GoalData.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/decision
)
_generate_msg_py(decision
  "/home/lian/robot_ws/src/decision/msg/UDPReceived.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/decision
)
_generate_msg_py(decision
  "/home/lian/robot_ws/src/decision/msg/gyro_euler.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/decision
)
_generate_msg_py(decision
  "/home/lian/robot_ws/src/decision/msg/Obstacle.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/decision
)
_generate_msg_py(decision
  "/home/lian/robot_ws/src/decision/msg/head_angle.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/decision
)
_generate_msg_py(decision
  "/home/lian/robot_ws/src/decision/msg/OutputData.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/decision
)
_generate_msg_py(decision
  "/home/lian/robot_ws/src/decision/msg/SerialReceived.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/decision
)

### Generating Services
_generate_srv_py(decision
  "/home/lian/robot_ws/src/decision/srv/head.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/decision
)
_generate_srv_py(decision
  "/home/lian/robot_ws/src/decision/srv/Pathplaning_the.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/decision
)
_generate_srv_py(decision
  "/home/lian/robot_ws/src/decision/srv/walk.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/decision
)

### Generating Module File
_generate_module_py(decision
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/decision
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(decision_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(decision_generate_messages decision_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lian/robot_ws/src/decision/msg/Ball.msg" NAME_WE)
add_dependencies(decision_generate_messages_py _decision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/decision/srv/head.srv" NAME_WE)
add_dependencies(decision_generate_messages_py _decision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/decision/msg/gameControl.msg" NAME_WE)
add_dependencies(decision_generate_messages_py _decision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/decision/msg/GoalData.msg" NAME_WE)
add_dependencies(decision_generate_messages_py _decision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/decision/srv/Pathplaning_the.srv" NAME_WE)
add_dependencies(decision_generate_messages_py _decision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/decision/msg/gyro_euler.msg" NAME_WE)
add_dependencies(decision_generate_messages_py _decision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/decision/msg/UDPReceived.msg" NAME_WE)
add_dependencies(decision_generate_messages_py _decision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/decision/msg/Obstacle.msg" NAME_WE)
add_dependencies(decision_generate_messages_py _decision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/decision/msg/head_angle.msg" NAME_WE)
add_dependencies(decision_generate_messages_py _decision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/decision/msg/OutputData.msg" NAME_WE)
add_dependencies(decision_generate_messages_py _decision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/decision/srv/walk.srv" NAME_WE)
add_dependencies(decision_generate_messages_py _decision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/decision/msg/SerialReceived.msg" NAME_WE)
add_dependencies(decision_generate_messages_py _decision_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(decision_genpy)
add_dependencies(decision_genpy decision_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS decision_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/decision)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/decision
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(decision_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(decision_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/decision)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/decision
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(decision_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(decision_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/decision)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/decision
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(decision_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(decision_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/decision)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/decision
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(decision_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(decision_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/decision)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/decision\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/decision
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(decision_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(decision_generate_messages_py std_msgs_generate_messages_py)
endif()
