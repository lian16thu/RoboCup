# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "head_motion: 2 messages, 1 services")

set(MSG_I_FLAGS "-Ihead_motion:/home/lian/robot_ws/src/head_motion/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(head_motion_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/lian/robot_ws/src/head_motion/srv/head_control.srv" NAME_WE)
add_custom_target(_head_motion_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "head_motion" "/home/lian/robot_ws/src/head_motion/srv/head_control.srv" ""
)

get_filename_component(_filename "/home/lian/robot_ws/src/head_motion/msg/head_pose.msg" NAME_WE)
add_custom_target(_head_motion_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "head_motion" "/home/lian/robot_ws/src/head_motion/msg/head_pose.msg" ""
)

get_filename_component(_filename "/home/lian/robot_ws/src/head_motion/msg/head_servo_angel.msg" NAME_WE)
add_custom_target(_head_motion_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "head_motion" "/home/lian/robot_ws/src/head_motion/msg/head_servo_angel.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(head_motion
  "/home/lian/robot_ws/src/head_motion/msg/head_pose.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/head_motion
)
_generate_msg_cpp(head_motion
  "/home/lian/robot_ws/src/head_motion/msg/head_servo_angel.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/head_motion
)

### Generating Services
_generate_srv_cpp(head_motion
  "/home/lian/robot_ws/src/head_motion/srv/head_control.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/head_motion
)

### Generating Module File
_generate_module_cpp(head_motion
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/head_motion
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(head_motion_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(head_motion_generate_messages head_motion_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lian/robot_ws/src/head_motion/srv/head_control.srv" NAME_WE)
add_dependencies(head_motion_generate_messages_cpp _head_motion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/head_motion/msg/head_pose.msg" NAME_WE)
add_dependencies(head_motion_generate_messages_cpp _head_motion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/head_motion/msg/head_servo_angel.msg" NAME_WE)
add_dependencies(head_motion_generate_messages_cpp _head_motion_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(head_motion_gencpp)
add_dependencies(head_motion_gencpp head_motion_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS head_motion_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(head_motion
  "/home/lian/robot_ws/src/head_motion/msg/head_pose.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/head_motion
)
_generate_msg_eus(head_motion
  "/home/lian/robot_ws/src/head_motion/msg/head_servo_angel.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/head_motion
)

### Generating Services
_generate_srv_eus(head_motion
  "/home/lian/robot_ws/src/head_motion/srv/head_control.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/head_motion
)

### Generating Module File
_generate_module_eus(head_motion
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/head_motion
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(head_motion_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(head_motion_generate_messages head_motion_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lian/robot_ws/src/head_motion/srv/head_control.srv" NAME_WE)
add_dependencies(head_motion_generate_messages_eus _head_motion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/head_motion/msg/head_pose.msg" NAME_WE)
add_dependencies(head_motion_generate_messages_eus _head_motion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/head_motion/msg/head_servo_angel.msg" NAME_WE)
add_dependencies(head_motion_generate_messages_eus _head_motion_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(head_motion_geneus)
add_dependencies(head_motion_geneus head_motion_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS head_motion_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(head_motion
  "/home/lian/robot_ws/src/head_motion/msg/head_pose.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/head_motion
)
_generate_msg_lisp(head_motion
  "/home/lian/robot_ws/src/head_motion/msg/head_servo_angel.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/head_motion
)

### Generating Services
_generate_srv_lisp(head_motion
  "/home/lian/robot_ws/src/head_motion/srv/head_control.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/head_motion
)

### Generating Module File
_generate_module_lisp(head_motion
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/head_motion
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(head_motion_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(head_motion_generate_messages head_motion_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lian/robot_ws/src/head_motion/srv/head_control.srv" NAME_WE)
add_dependencies(head_motion_generate_messages_lisp _head_motion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/head_motion/msg/head_pose.msg" NAME_WE)
add_dependencies(head_motion_generate_messages_lisp _head_motion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/head_motion/msg/head_servo_angel.msg" NAME_WE)
add_dependencies(head_motion_generate_messages_lisp _head_motion_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(head_motion_genlisp)
add_dependencies(head_motion_genlisp head_motion_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS head_motion_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(head_motion
  "/home/lian/robot_ws/src/head_motion/msg/head_pose.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/head_motion
)
_generate_msg_nodejs(head_motion
  "/home/lian/robot_ws/src/head_motion/msg/head_servo_angel.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/head_motion
)

### Generating Services
_generate_srv_nodejs(head_motion
  "/home/lian/robot_ws/src/head_motion/srv/head_control.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/head_motion
)

### Generating Module File
_generate_module_nodejs(head_motion
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/head_motion
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(head_motion_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(head_motion_generate_messages head_motion_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lian/robot_ws/src/head_motion/srv/head_control.srv" NAME_WE)
add_dependencies(head_motion_generate_messages_nodejs _head_motion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/head_motion/msg/head_pose.msg" NAME_WE)
add_dependencies(head_motion_generate_messages_nodejs _head_motion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/head_motion/msg/head_servo_angel.msg" NAME_WE)
add_dependencies(head_motion_generate_messages_nodejs _head_motion_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(head_motion_gennodejs)
add_dependencies(head_motion_gennodejs head_motion_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS head_motion_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(head_motion
  "/home/lian/robot_ws/src/head_motion/msg/head_pose.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/head_motion
)
_generate_msg_py(head_motion
  "/home/lian/robot_ws/src/head_motion/msg/head_servo_angel.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/head_motion
)

### Generating Services
_generate_srv_py(head_motion
  "/home/lian/robot_ws/src/head_motion/srv/head_control.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/head_motion
)

### Generating Module File
_generate_module_py(head_motion
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/head_motion
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(head_motion_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(head_motion_generate_messages head_motion_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lian/robot_ws/src/head_motion/srv/head_control.srv" NAME_WE)
add_dependencies(head_motion_generate_messages_py _head_motion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/head_motion/msg/head_pose.msg" NAME_WE)
add_dependencies(head_motion_generate_messages_py _head_motion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/head_motion/msg/head_servo_angel.msg" NAME_WE)
add_dependencies(head_motion_generate_messages_py _head_motion_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(head_motion_genpy)
add_dependencies(head_motion_genpy head_motion_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS head_motion_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/head_motion)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/head_motion
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(head_motion_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/head_motion)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/head_motion
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(head_motion_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/head_motion)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/head_motion
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(head_motion_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/head_motion)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/head_motion
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(head_motion_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/head_motion)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/head_motion\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/head_motion
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(head_motion_generate_messages_py std_msgs_generate_messages_py)
endif()
