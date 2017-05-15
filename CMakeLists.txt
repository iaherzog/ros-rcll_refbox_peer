cmake_minimum_required(VERSION 2.8.3)
project(rcll_refbox_peer)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rcll_ros_msgs
  roscpp
	tf2
)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

include(CheckCXXCompilerFlag)

# Do not do this atm, need to find llsf_msgs
#include(FindPkgConfig)
#pkg_check_modules(protobuf_comm protobuf_comm)

if (protobuf_comm_FOUND)
	CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
	CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
	if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
	elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
	else()
		message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
	endif()
else()
	find_package(Fawkes COMPONENTS protobuf_comm llsf_msgs)
	if (NOT Fawkes_FOUND)
		find_package(RCLLRefbox COMPONENTS protobuf_comm llsf_msgs)
		if (NOT RCLLRefbox_FOUND)
			message(FATAL_ERROR "protobuf_comm not found, tried system, Fawkes, and RCLL refbox")
		endif()
	endif()
endif()

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rcll_refbox_peer
#  CATKIN_DEPENDS rcll_msgs roscpp
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

## Declare a C++ library
# add_library(rcll_refbox_peer
#   src/${PROJECT_NAME}/rcll_refbox_peer.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(rcll_refbox_peer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(rcll_refbox_peer src/rcll_refbox_peer_node.cpp)
add_dependencies(rcll_refbox_peer ${catkin_EXPORTED_TARGETS})

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(rcll_refbox_peer_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

if (protobuf_comm_FOUND)
  message(STATUS "Using system protobuf_comm")
	set_target_properties(rcll_refbox_peer PROPERTIES
		INCLUDE_DIRECTORIES "${protobuf_comm_INCLUDE_DIRS}"
		COMPILE_FLAGS "${protobuf_comm_CFLAGS}"
    LINK_FLAGS "${protobuf_comm_LFLAGS}"
    LINK_LIBRARIES "${protobuf_comm_LIBRARIES};${catkin_LIBRARIES}"
	)

elseif (Fawkes_FOUND)
  list(APPEND Fawkes_LIBRARIES "llsf_msgs")

	# Use Fawkes' protobuf_comm
  set_target_properties(rcll_refbox_peer PROPERTIES
    INCLUDE_DIRECTORIES "${Fawkes_INCLUDE_DIRS};${catkin_INCLUDE_DIRS}"
    COMPILE_FLAGS "${Fawkes_CFLAGS} ${catkin_CFLAGS}"
    LINK_FLAGS "${Fawkes_LFLAGS}"
    LINK_LIBRARIES "${Fawkes_LIBRARIES};${catkin_LIBRARIES}"
    INSTALL_RPATH "${Fawkes_LIBRARY_DIRS}"
    INSTALL_RPATH_USE_LINK_PATH TRUE
    BUILD_WITH_INSTALL_RPATH TRUE)
  message(STATUS "Using Fawkes protobuf_comm")

elseif (RCLLRefbox_FOUND)
  list(APPEND _LIBRARIES "llsf_msgs")

	# Use Fawkes' protobuf_comm
  set_target_properties(rcll_refbox_peer PROPERTIES
    INCLUDE_DIRECTORIES "${Fawkes_INCLUDE_DIRS};${catkin_INCLUDE_DIRS}"
    COMPILE_FLAGS "${Fawkes_CFLAGS} ${catkin_CFLAGS}"
    LINK_FLAGS "${Fawkes_LFLAGS}"
    LINK_LIBRARIES "${Fawkes_LIBRARIES};${catkin_LIBRARIES}"
    INSTALL_RPATH "${Fawkes_LIBRARY_DIRS}"
    INSTALL_RPATH_USE_LINK_PATH TRUE
    BUILD_WITH_INSTALL_RPATH TRUE)
  message(STATUS "Using Fawkes protobuf_comm")
else()
  message(FATAL_ERROR "Neither system nor Fawkes protobuf_comm found")
endif()

#############
## Install ##
#############

install(TARGETS rcll_refbox_peer
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install launch files
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/
)

