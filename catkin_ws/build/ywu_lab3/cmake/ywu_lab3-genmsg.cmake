# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "ywu_lab3: 0 messages, 1 services")

set(MSG_I_FLAGS "-Inav_msgs:/opt/ros/indigo/share/nav_msgs/cmake/../msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/indigo/share/geometry_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/indigo/share/actionlib_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(ywu_lab3_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/rbe/catkin_ws/src/ywu_lab3/srv/aStar.srv" NAME_WE)
add_custom_target(_ywu_lab3_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ywu_lab3" "/home/rbe/catkin_ws/src/ywu_lab3/srv/aStar.srv" "geometry_msgs/Point:std_msgs/Header:geometry_msgs/Quaternion:geometry_msgs/PoseStamped:nav_msgs/Path:geometry_msgs/Pose"
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(ywu_lab3
  "/home/rbe/catkin_ws/src/ywu_lab3/srv/aStar.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/indigo/share/nav_msgs/cmake/../msg/Path.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ywu_lab3
)

### Generating Module File
_generate_module_cpp(ywu_lab3
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ywu_lab3
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(ywu_lab3_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(ywu_lab3_generate_messages ywu_lab3_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rbe/catkin_ws/src/ywu_lab3/srv/aStar.srv" NAME_WE)
add_dependencies(ywu_lab3_generate_messages_cpp _ywu_lab3_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ywu_lab3_gencpp)
add_dependencies(ywu_lab3_gencpp ywu_lab3_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ywu_lab3_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(ywu_lab3
  "/home/rbe/catkin_ws/src/ywu_lab3/srv/aStar.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/indigo/share/nav_msgs/cmake/../msg/Path.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ywu_lab3
)

### Generating Module File
_generate_module_lisp(ywu_lab3
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ywu_lab3
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(ywu_lab3_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(ywu_lab3_generate_messages ywu_lab3_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rbe/catkin_ws/src/ywu_lab3/srv/aStar.srv" NAME_WE)
add_dependencies(ywu_lab3_generate_messages_lisp _ywu_lab3_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ywu_lab3_genlisp)
add_dependencies(ywu_lab3_genlisp ywu_lab3_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ywu_lab3_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(ywu_lab3
  "/home/rbe/catkin_ws/src/ywu_lab3/srv/aStar.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/indigo/share/nav_msgs/cmake/../msg/Path.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ywu_lab3
)

### Generating Module File
_generate_module_py(ywu_lab3
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ywu_lab3
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(ywu_lab3_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(ywu_lab3_generate_messages ywu_lab3_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rbe/catkin_ws/src/ywu_lab3/srv/aStar.srv" NAME_WE)
add_dependencies(ywu_lab3_generate_messages_py _ywu_lab3_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ywu_lab3_genpy)
add_dependencies(ywu_lab3_genpy ywu_lab3_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ywu_lab3_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ywu_lab3)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ywu_lab3
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(ywu_lab3_generate_messages_cpp nav_msgs_generate_messages_cpp)
add_dependencies(ywu_lab3_generate_messages_cpp std_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ywu_lab3)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ywu_lab3
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(ywu_lab3_generate_messages_lisp nav_msgs_generate_messages_lisp)
add_dependencies(ywu_lab3_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ywu_lab3)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ywu_lab3\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ywu_lab3
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(ywu_lab3_generate_messages_py nav_msgs_generate_messages_py)
add_dependencies(ywu_lab3_generate_messages_py std_msgs_generate_messages_py)
