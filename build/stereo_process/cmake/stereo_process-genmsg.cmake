# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "stereo_process: 4 messages, 1 services")

set(MSG_I_FLAGS "-Istereo_process:/home/lian/robot_ws/src/stereo_process/msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/kinetic/share/sensor_msgs/cmake/../msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(stereo_process_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/lian/robot_ws/src/stereo_process/msg/ObjectOnImage.msg" NAME_WE)
add_custom_target(_stereo_process_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "stereo_process" "/home/lian/robot_ws/src/stereo_process/msg/ObjectOnImage.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/lian/robot_ws/src/stereo_process/msg/Ball.msg" NAME_WE)
add_custom_target(_stereo_process_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "stereo_process" "/home/lian/robot_ws/src/stereo_process/msg/Ball.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/lian/robot_ws/src/stereo_process/msg/Obstacles.msg" NAME_WE)
add_custom_target(_stereo_process_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "stereo_process" "/home/lian/robot_ws/src/stereo_process/msg/Obstacles.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/lian/robot_ws/src/stereo_process/msg/Goalpost.msg" NAME_WE)
add_custom_target(_stereo_process_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "stereo_process" "/home/lian/robot_ws/src/stereo_process/msg/Goalpost.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/lian/robot_ws/src/stereo_process/srv/DepthRequest.srv" NAME_WE)
add_custom_target(_stereo_process_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "stereo_process" "/home/lian/robot_ws/src/stereo_process/srv/DepthRequest.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(stereo_process
  "/home/lian/robot_ws/src/stereo_process/msg/ObjectOnImage.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/stereo_process
)
_generate_msg_cpp(stereo_process
  "/home/lian/robot_ws/src/stereo_process/msg/Ball.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/stereo_process
)
_generate_msg_cpp(stereo_process
  "/home/lian/robot_ws/src/stereo_process/msg/Obstacles.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/stereo_process
)
_generate_msg_cpp(stereo_process
  "/home/lian/robot_ws/src/stereo_process/msg/Goalpost.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/stereo_process
)

### Generating Services
_generate_srv_cpp(stereo_process
  "/home/lian/robot_ws/src/stereo_process/srv/DepthRequest.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/stereo_process
)

### Generating Module File
_generate_module_cpp(stereo_process
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/stereo_process
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(stereo_process_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(stereo_process_generate_messages stereo_process_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lian/robot_ws/src/stereo_process/msg/ObjectOnImage.msg" NAME_WE)
add_dependencies(stereo_process_generate_messages_cpp _stereo_process_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/stereo_process/msg/Ball.msg" NAME_WE)
add_dependencies(stereo_process_generate_messages_cpp _stereo_process_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/stereo_process/msg/Obstacles.msg" NAME_WE)
add_dependencies(stereo_process_generate_messages_cpp _stereo_process_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/stereo_process/msg/Goalpost.msg" NAME_WE)
add_dependencies(stereo_process_generate_messages_cpp _stereo_process_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/stereo_process/srv/DepthRequest.srv" NAME_WE)
add_dependencies(stereo_process_generate_messages_cpp _stereo_process_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(stereo_process_gencpp)
add_dependencies(stereo_process_gencpp stereo_process_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS stereo_process_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(stereo_process
  "/home/lian/robot_ws/src/stereo_process/msg/ObjectOnImage.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/stereo_process
)
_generate_msg_eus(stereo_process
  "/home/lian/robot_ws/src/stereo_process/msg/Ball.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/stereo_process
)
_generate_msg_eus(stereo_process
  "/home/lian/robot_ws/src/stereo_process/msg/Obstacles.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/stereo_process
)
_generate_msg_eus(stereo_process
  "/home/lian/robot_ws/src/stereo_process/msg/Goalpost.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/stereo_process
)

### Generating Services
_generate_srv_eus(stereo_process
  "/home/lian/robot_ws/src/stereo_process/srv/DepthRequest.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/stereo_process
)

### Generating Module File
_generate_module_eus(stereo_process
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/stereo_process
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(stereo_process_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(stereo_process_generate_messages stereo_process_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lian/robot_ws/src/stereo_process/msg/ObjectOnImage.msg" NAME_WE)
add_dependencies(stereo_process_generate_messages_eus _stereo_process_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/stereo_process/msg/Ball.msg" NAME_WE)
add_dependencies(stereo_process_generate_messages_eus _stereo_process_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/stereo_process/msg/Obstacles.msg" NAME_WE)
add_dependencies(stereo_process_generate_messages_eus _stereo_process_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/stereo_process/msg/Goalpost.msg" NAME_WE)
add_dependencies(stereo_process_generate_messages_eus _stereo_process_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/stereo_process/srv/DepthRequest.srv" NAME_WE)
add_dependencies(stereo_process_generate_messages_eus _stereo_process_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(stereo_process_geneus)
add_dependencies(stereo_process_geneus stereo_process_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS stereo_process_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(stereo_process
  "/home/lian/robot_ws/src/stereo_process/msg/ObjectOnImage.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/stereo_process
)
_generate_msg_lisp(stereo_process
  "/home/lian/robot_ws/src/stereo_process/msg/Ball.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/stereo_process
)
_generate_msg_lisp(stereo_process
  "/home/lian/robot_ws/src/stereo_process/msg/Obstacles.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/stereo_process
)
_generate_msg_lisp(stereo_process
  "/home/lian/robot_ws/src/stereo_process/msg/Goalpost.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/stereo_process
)

### Generating Services
_generate_srv_lisp(stereo_process
  "/home/lian/robot_ws/src/stereo_process/srv/DepthRequest.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/stereo_process
)

### Generating Module File
_generate_module_lisp(stereo_process
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/stereo_process
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(stereo_process_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(stereo_process_generate_messages stereo_process_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lian/robot_ws/src/stereo_process/msg/ObjectOnImage.msg" NAME_WE)
add_dependencies(stereo_process_generate_messages_lisp _stereo_process_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/stereo_process/msg/Ball.msg" NAME_WE)
add_dependencies(stereo_process_generate_messages_lisp _stereo_process_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/stereo_process/msg/Obstacles.msg" NAME_WE)
add_dependencies(stereo_process_generate_messages_lisp _stereo_process_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/stereo_process/msg/Goalpost.msg" NAME_WE)
add_dependencies(stereo_process_generate_messages_lisp _stereo_process_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/stereo_process/srv/DepthRequest.srv" NAME_WE)
add_dependencies(stereo_process_generate_messages_lisp _stereo_process_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(stereo_process_genlisp)
add_dependencies(stereo_process_genlisp stereo_process_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS stereo_process_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(stereo_process
  "/home/lian/robot_ws/src/stereo_process/msg/ObjectOnImage.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/stereo_process
)
_generate_msg_nodejs(stereo_process
  "/home/lian/robot_ws/src/stereo_process/msg/Ball.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/stereo_process
)
_generate_msg_nodejs(stereo_process
  "/home/lian/robot_ws/src/stereo_process/msg/Obstacles.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/stereo_process
)
_generate_msg_nodejs(stereo_process
  "/home/lian/robot_ws/src/stereo_process/msg/Goalpost.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/stereo_process
)

### Generating Services
_generate_srv_nodejs(stereo_process
  "/home/lian/robot_ws/src/stereo_process/srv/DepthRequest.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/stereo_process
)

### Generating Module File
_generate_module_nodejs(stereo_process
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/stereo_process
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(stereo_process_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(stereo_process_generate_messages stereo_process_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lian/robot_ws/src/stereo_process/msg/ObjectOnImage.msg" NAME_WE)
add_dependencies(stereo_process_generate_messages_nodejs _stereo_process_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/stereo_process/msg/Ball.msg" NAME_WE)
add_dependencies(stereo_process_generate_messages_nodejs _stereo_process_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/stereo_process/msg/Obstacles.msg" NAME_WE)
add_dependencies(stereo_process_generate_messages_nodejs _stereo_process_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/stereo_process/msg/Goalpost.msg" NAME_WE)
add_dependencies(stereo_process_generate_messages_nodejs _stereo_process_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/stereo_process/srv/DepthRequest.srv" NAME_WE)
add_dependencies(stereo_process_generate_messages_nodejs _stereo_process_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(stereo_process_gennodejs)
add_dependencies(stereo_process_gennodejs stereo_process_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS stereo_process_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(stereo_process
  "/home/lian/robot_ws/src/stereo_process/msg/ObjectOnImage.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/stereo_process
)
_generate_msg_py(stereo_process
  "/home/lian/robot_ws/src/stereo_process/msg/Ball.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/stereo_process
)
_generate_msg_py(stereo_process
  "/home/lian/robot_ws/src/stereo_process/msg/Obstacles.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/stereo_process
)
_generate_msg_py(stereo_process
  "/home/lian/robot_ws/src/stereo_process/msg/Goalpost.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/stereo_process
)

### Generating Services
_generate_srv_py(stereo_process
  "/home/lian/robot_ws/src/stereo_process/srv/DepthRequest.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/stereo_process
)

### Generating Module File
_generate_module_py(stereo_process
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/stereo_process
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(stereo_process_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(stereo_process_generate_messages stereo_process_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lian/robot_ws/src/stereo_process/msg/ObjectOnImage.msg" NAME_WE)
add_dependencies(stereo_process_generate_messages_py _stereo_process_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/stereo_process/msg/Ball.msg" NAME_WE)
add_dependencies(stereo_process_generate_messages_py _stereo_process_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/stereo_process/msg/Obstacles.msg" NAME_WE)
add_dependencies(stereo_process_generate_messages_py _stereo_process_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/stereo_process/msg/Goalpost.msg" NAME_WE)
add_dependencies(stereo_process_generate_messages_py _stereo_process_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lian/robot_ws/src/stereo_process/srv/DepthRequest.srv" NAME_WE)
add_dependencies(stereo_process_generate_messages_py _stereo_process_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(stereo_process_genpy)
add_dependencies(stereo_process_genpy stereo_process_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS stereo_process_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/stereo_process)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/stereo_process
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(stereo_process_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(stereo_process_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(stereo_process_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/stereo_process)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/stereo_process
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(stereo_process_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(stereo_process_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(stereo_process_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/stereo_process)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/stereo_process
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(stereo_process_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(stereo_process_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(stereo_process_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/stereo_process)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/stereo_process
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(stereo_process_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(stereo_process_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(stereo_process_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/stereo_process)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/stereo_process\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/stereo_process
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(stereo_process_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(stereo_process_generate_messages_py sensor_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(stereo_process_generate_messages_py std_msgs_generate_messages_py)
endif()
