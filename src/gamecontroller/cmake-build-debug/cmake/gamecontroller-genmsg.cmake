# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "gamecontroller: 1 messages, 0 services")

set(MSG_I_FLAGS "-Igamecontroller:/home/liuyuezhangadam/Desktop/gamecontroller/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(gamecontroller_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/liuyuezhangadam/Desktop/gamecontroller/msg/gameControl.msg" NAME_WE)
add_custom_target(_gamecontroller_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "gamecontroller" "/home/liuyuezhangadam/Desktop/gamecontroller/msg/gameControl.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(gamecontroller
  "/home/liuyuezhangadam/Desktop/gamecontroller/msg/gameControl.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/gamecontroller
)

### Generating Services

### Generating Module File
_generate_module_cpp(gamecontroller
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/gamecontroller
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(gamecontroller_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(gamecontroller_generate_messages gamecontroller_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/liuyuezhangadam/Desktop/gamecontroller/msg/gameControl.msg" NAME_WE)
add_dependencies(gamecontroller_generate_messages_cpp _gamecontroller_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(gamecontroller_gencpp)
add_dependencies(gamecontroller_gencpp gamecontroller_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS gamecontroller_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(gamecontroller
  "/home/liuyuezhangadam/Desktop/gamecontroller/msg/gameControl.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/gamecontroller
)

### Generating Services

### Generating Module File
_generate_module_eus(gamecontroller
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/gamecontroller
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(gamecontroller_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(gamecontroller_generate_messages gamecontroller_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/liuyuezhangadam/Desktop/gamecontroller/msg/gameControl.msg" NAME_WE)
add_dependencies(gamecontroller_generate_messages_eus _gamecontroller_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(gamecontroller_geneus)
add_dependencies(gamecontroller_geneus gamecontroller_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS gamecontroller_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(gamecontroller
  "/home/liuyuezhangadam/Desktop/gamecontroller/msg/gameControl.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/gamecontroller
)

### Generating Services

### Generating Module File
_generate_module_lisp(gamecontroller
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/gamecontroller
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(gamecontroller_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(gamecontroller_generate_messages gamecontroller_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/liuyuezhangadam/Desktop/gamecontroller/msg/gameControl.msg" NAME_WE)
add_dependencies(gamecontroller_generate_messages_lisp _gamecontroller_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(gamecontroller_genlisp)
add_dependencies(gamecontroller_genlisp gamecontroller_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS gamecontroller_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(gamecontroller
  "/home/liuyuezhangadam/Desktop/gamecontroller/msg/gameControl.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/gamecontroller
)

### Generating Services

### Generating Module File
_generate_module_nodejs(gamecontroller
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/gamecontroller
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(gamecontroller_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(gamecontroller_generate_messages gamecontroller_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/liuyuezhangadam/Desktop/gamecontroller/msg/gameControl.msg" NAME_WE)
add_dependencies(gamecontroller_generate_messages_nodejs _gamecontroller_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(gamecontroller_gennodejs)
add_dependencies(gamecontroller_gennodejs gamecontroller_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS gamecontroller_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(gamecontroller
  "/home/liuyuezhangadam/Desktop/gamecontroller/msg/gameControl.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gamecontroller
)

### Generating Services

### Generating Module File
_generate_module_py(gamecontroller
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gamecontroller
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(gamecontroller_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(gamecontroller_generate_messages gamecontroller_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/liuyuezhangadam/Desktop/gamecontroller/msg/gameControl.msg" NAME_WE)
add_dependencies(gamecontroller_generate_messages_py _gamecontroller_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(gamecontroller_genpy)
add_dependencies(gamecontroller_genpy gamecontroller_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS gamecontroller_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/gamecontroller)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/gamecontroller
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(gamecontroller_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/gamecontroller)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/gamecontroller
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(gamecontroller_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/gamecontroller)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/gamecontroller
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(gamecontroller_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/gamecontroller)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/gamecontroller
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(gamecontroller_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gamecontroller)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gamecontroller\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gamecontroller
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(gamecontroller_generate_messages_py std_msgs_generate_messages_py)
endif()
