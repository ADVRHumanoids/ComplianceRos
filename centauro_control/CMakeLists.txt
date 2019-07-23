cmake_minimum_required(VERSION 2.8.3)
project(centauro_control)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)# COMPONENTS
##  ros_control
##  ros_controllers
##)
set(PYTHON_INSTALL "")

FUNCTION( prepend_paths SOURCE_FILES PARENT_DIR)
  FOREACH( SOURCE_FILE ${${SOURCE_FILES}} )
    SET( MODIFIED ${MODIFIED} ${PARENT_DIR}/${SOURCE_FILE} )
  ENDFOREACH()
  SET( ${SOURCE_FILES} ${MODIFIED} PARENT_SCOPE )
ENDFUNCTION()

#LIST( APPEND PYTHON_INSTALL colliding_forearms collision_avoidance_init mammal_like moving_arms moving_arms_varying_frequencies new_mammal_like raise_rear_leg spider spider_2 standing_position velocity_mammal_like wheels zero )
#prepend_paths(PYTHON_INSTALL compliance)
LIST( APPEND PYTHON_INSTALL wheels zero mammal_1 mammal_2 mammal_3 spider)
prepend_paths(PYTHON_INSTALL compliance)

LIST( APPEND PYTHON_INSTALL collision_test computeErrors log_data)

catkin_package(
 INCLUDE_DIRS nodes
#  LIBRARIES centauro_control
#  CATKIN_DEPENDS ros_control ros_controllers
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)


#############
## Install ##
#############

## Mark cpp header files for installation
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN ".svn" EXCLUDE)

MESSAGE(WARNING "PYTHON INSTAL: " ${PYTHON_INSTALL})
MESSAGE(WARNING "CATKIN_PACKAGE_BIN_DESTINATION: " ${CATKIN_PACKAGE_SHARE_DESTINATION})

prepend_paths(PYTHON_INSTALL nodes)

catkin_install_python(PROGRAMS ${PYTHON_INSTALL}
                      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
