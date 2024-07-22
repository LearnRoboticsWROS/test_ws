# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "position_tracker: 0 messages, 1 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(position_tracker_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/ros/test_ws/src/position_tracker/srv/GetPosition.srv" NAME_WE)
add_custom_target(_position_tracker_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "position_tracker" "/home/ros/test_ws/src/position_tracker/srv/GetPosition.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(position_tracker
  "/home/ros/test_ws/src/position_tracker/srv/GetPosition.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/position_tracker
)

### Generating Module File
_generate_module_cpp(position_tracker
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/position_tracker
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(position_tracker_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(position_tracker_generate_messages position_tracker_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ros/test_ws/src/position_tracker/srv/GetPosition.srv" NAME_WE)
add_dependencies(position_tracker_generate_messages_cpp _position_tracker_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(position_tracker_gencpp)
add_dependencies(position_tracker_gencpp position_tracker_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS position_tracker_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(position_tracker
  "/home/ros/test_ws/src/position_tracker/srv/GetPosition.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/position_tracker
)

### Generating Module File
_generate_module_eus(position_tracker
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/position_tracker
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(position_tracker_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(position_tracker_generate_messages position_tracker_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ros/test_ws/src/position_tracker/srv/GetPosition.srv" NAME_WE)
add_dependencies(position_tracker_generate_messages_eus _position_tracker_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(position_tracker_geneus)
add_dependencies(position_tracker_geneus position_tracker_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS position_tracker_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(position_tracker
  "/home/ros/test_ws/src/position_tracker/srv/GetPosition.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/position_tracker
)

### Generating Module File
_generate_module_lisp(position_tracker
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/position_tracker
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(position_tracker_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(position_tracker_generate_messages position_tracker_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ros/test_ws/src/position_tracker/srv/GetPosition.srv" NAME_WE)
add_dependencies(position_tracker_generate_messages_lisp _position_tracker_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(position_tracker_genlisp)
add_dependencies(position_tracker_genlisp position_tracker_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS position_tracker_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(position_tracker
  "/home/ros/test_ws/src/position_tracker/srv/GetPosition.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/position_tracker
)

### Generating Module File
_generate_module_nodejs(position_tracker
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/position_tracker
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(position_tracker_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(position_tracker_generate_messages position_tracker_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ros/test_ws/src/position_tracker/srv/GetPosition.srv" NAME_WE)
add_dependencies(position_tracker_generate_messages_nodejs _position_tracker_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(position_tracker_gennodejs)
add_dependencies(position_tracker_gennodejs position_tracker_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS position_tracker_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(position_tracker
  "/home/ros/test_ws/src/position_tracker/srv/GetPosition.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/position_tracker
)

### Generating Module File
_generate_module_py(position_tracker
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/position_tracker
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(position_tracker_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(position_tracker_generate_messages position_tracker_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ros/test_ws/src/position_tracker/srv/GetPosition.srv" NAME_WE)
add_dependencies(position_tracker_generate_messages_py _position_tracker_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(position_tracker_genpy)
add_dependencies(position_tracker_genpy position_tracker_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS position_tracker_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/position_tracker)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/position_tracker
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(position_tracker_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/position_tracker)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/position_tracker
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(position_tracker_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/position_tracker)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/position_tracker
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(position_tracker_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/position_tracker)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/position_tracker
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(position_tracker_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/position_tracker)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/position_tracker\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/position_tracker
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(position_tracker_generate_messages_py std_msgs_generate_messages_py)
endif()
