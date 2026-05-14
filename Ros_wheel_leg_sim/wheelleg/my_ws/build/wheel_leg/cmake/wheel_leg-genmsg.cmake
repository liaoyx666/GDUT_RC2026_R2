# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "wheel_leg: 1 messages, 0 services")

set(MSG_I_FLAGS "-Iwheel_leg:/home/xu/my_ws/src/wheel_leg/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(wheel_leg_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/xu/my_ws/src/wheel_leg/msg/wl_control.msg" NAME_WE)
add_custom_target(_wheel_leg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "wheel_leg" "/home/xu/my_ws/src/wheel_leg/msg/wl_control.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(wheel_leg
  "/home/xu/my_ws/src/wheel_leg/msg/wl_control.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/wheel_leg
)

### Generating Services

### Generating Module File
_generate_module_cpp(wheel_leg
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/wheel_leg
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(wheel_leg_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(wheel_leg_generate_messages wheel_leg_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/xu/my_ws/src/wheel_leg/msg/wl_control.msg" NAME_WE)
add_dependencies(wheel_leg_generate_messages_cpp _wheel_leg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(wheel_leg_gencpp)
add_dependencies(wheel_leg_gencpp wheel_leg_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS wheel_leg_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(wheel_leg
  "/home/xu/my_ws/src/wheel_leg/msg/wl_control.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/wheel_leg
)

### Generating Services

### Generating Module File
_generate_module_eus(wheel_leg
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/wheel_leg
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(wheel_leg_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(wheel_leg_generate_messages wheel_leg_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/xu/my_ws/src/wheel_leg/msg/wl_control.msg" NAME_WE)
add_dependencies(wheel_leg_generate_messages_eus _wheel_leg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(wheel_leg_geneus)
add_dependencies(wheel_leg_geneus wheel_leg_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS wheel_leg_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(wheel_leg
  "/home/xu/my_ws/src/wheel_leg/msg/wl_control.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/wheel_leg
)

### Generating Services

### Generating Module File
_generate_module_lisp(wheel_leg
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/wheel_leg
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(wheel_leg_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(wheel_leg_generate_messages wheel_leg_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/xu/my_ws/src/wheel_leg/msg/wl_control.msg" NAME_WE)
add_dependencies(wheel_leg_generate_messages_lisp _wheel_leg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(wheel_leg_genlisp)
add_dependencies(wheel_leg_genlisp wheel_leg_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS wheel_leg_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(wheel_leg
  "/home/xu/my_ws/src/wheel_leg/msg/wl_control.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/wheel_leg
)

### Generating Services

### Generating Module File
_generate_module_nodejs(wheel_leg
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/wheel_leg
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(wheel_leg_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(wheel_leg_generate_messages wheel_leg_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/xu/my_ws/src/wheel_leg/msg/wl_control.msg" NAME_WE)
add_dependencies(wheel_leg_generate_messages_nodejs _wheel_leg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(wheel_leg_gennodejs)
add_dependencies(wheel_leg_gennodejs wheel_leg_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS wheel_leg_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(wheel_leg
  "/home/xu/my_ws/src/wheel_leg/msg/wl_control.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/wheel_leg
)

### Generating Services

### Generating Module File
_generate_module_py(wheel_leg
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/wheel_leg
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(wheel_leg_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(wheel_leg_generate_messages wheel_leg_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/xu/my_ws/src/wheel_leg/msg/wl_control.msg" NAME_WE)
add_dependencies(wheel_leg_generate_messages_py _wheel_leg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(wheel_leg_genpy)
add_dependencies(wheel_leg_genpy wheel_leg_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS wheel_leg_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/wheel_leg)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/wheel_leg
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(wheel_leg_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/wheel_leg)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/wheel_leg
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(wheel_leg_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/wheel_leg)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/wheel_leg
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(wheel_leg_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/wheel_leg)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/wheel_leg
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(wheel_leg_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/wheel_leg)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/wheel_leg\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/wheel_leg
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(wheel_leg_generate_messages_py std_msgs_generate_messages_py)
endif()
