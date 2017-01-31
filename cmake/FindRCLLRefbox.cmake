# - Find the RoboCup Logistics League Referee Box (http://www.robocup-logistics.org/refbox)
#
# Once done this will define
#
#  RCLLRefbox_FOUND - RCLL referee box was found
#  RCLLRefbox_INCLUDE_DIRS - RCLL include directories
#  RCLLRefbox_LIBRARY_DIRS - Link directories for RCLL libraries


# Copyright (c) 2015-2ß17  Tim Niemueller [www.niemueller.de]
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. The name of the author may not be used to endorse or promote products
#    derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
# IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
# OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
# IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
# NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
# THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#set(RCLLRB_DIR "$ENV{HOME}/robotics/tino-fawkes")

# Need that for many components
include(FindPkgConfig)

message(STATUS "Checking for RCLL refbox")

# These are components for which we do not need extra modules which
# only require basic cflags and ldflags
set(RCLLRB_BASE_COMPONENTS core)

if (NOT RCLLRB_DIR)
  if (NOT "$ENV{RCLLRB_DIR}" STREQUAL "")
    #message(STATUS "  checking env var dir $ENV{FAWKES_DIR}")
    # if the FAWKES_DIR environment variable is set first look there
    find_path(RCLLRB_DIR version.h
      PATHS ENV RCLLRB_DIR
      PATH_SUFFIXES src/libs/core
      NO_DEFAULT_PATH)
  endif()

  if (NOT RCLLRB_DIR)
    # try to find RCLL refbox in a few common paths
    find_path(RCLLRB_DIR
      version.h
      PATHS $ENV{HOME}/llsf-refbox $ENV{HOME}/rcll-refbox $ENV{HOME}/rci-refbox
            $ENV{HOME}/robotics/llsf-refbox $ENV{HOME}/robotics/rcll-refbox $ENV{HOME}/robotics/rci-refbox
      PATH_SUFFIXES src/libs/core
    )
  endif()
endif()

if (RCLLRB_DIR)
  if (RCLLRB_DIR MATCHES "src/libs/core$")
    get_filename_component(RCLLRB_DIR "${RCLLRB_DIR}/../../.." ABSOLUTE)
  endif()
  get_filename_component(RCLLRefbox_BASEDIR "${RCLLRB_DIR}" ABSOLUTE)

  message(STATUS "  base dir: ${RCLLRefbox_BASEDIR}")
endif()

if (RCLLRefbox_BASEDIR)
  # parse version
  file(STRINGS "${RCLLRB_DIR}/src/libs/core/version.h" RCLLRB_VERSION_FILE)
  foreach (LINE ${RCLLRB_VERSION_FILE})
    string(REGEX MATCH "^#define FAWKES_VERSION_MAJOR  ([0-9]+)$" MATCH "${LINE}")
    if (MATCH)
      set(RCLLRefbox_VERSION_MAJOR ${CMAKE_MATCH_1})
    endif()
    string(REGEX MATCH "^#define FAWKES_VERSION_MINOR  ([0-9]+)$" MATCH "${LINE}")
    if (MATCH)
      set(RCLLRefbox_VERSION_MINOR ${CMAKE_MATCH_1})
    endif()
    string(REGEX MATCH "^#define FAWKES_VERSION_MICRO  ([0-9]+)$" MATCH "${LINE}")
    if (MATCH)
      set(RCLLRefbox_VERSION_MICRO ${CMAKE_MATCH_1})
    endif()
  endforeach()
  message(STATUS "  version: ${RCLLRefbox_VERSION_MAJOR}.${RCLLRefbox_VERSION_MINOR}.${RCLLRefbox_VERSION_MICRO}")

  set(RCLLRefbox_FOUND TRUE)
  set(RCLLRefbox_BINDIR ${RCLLRefbox_BASEDIR}/bin)
  set(RCLLRefbox_LIBDIR ${RCLLRefbox_BASEDIR}/lib)
  set(RCLLRefbox_RESDIR ${RCLLRefbox_BASEDIR}/res)
  set(RCLLRefbox_CONFDIR ${RCLLRefbox_BASEDIR}/cfg)
  set(RCLLRefbox_PLUGINDIR ${RCLLRefbox_BASEDIR}/plugins)
  set(RCLLRefbox_TMPDIR "/tmp")
  set(RCLLRefbox_LOGDIR ${RCLLRefbox_BASEDIR}/log)
  set(RCLLRefbox_INCLUDE_DIRS
    ${RCLLRefbox_BASEDIR}/src ${RCLLRefbox_COREDIR}/src
    ${RCLLRefbox_BASEDIR}/src/libs ${RCLLRefbox_COREDIR}/src/libs)
  set(RCLLRefbox_LIBRARY_DIRS ${RCLLRefbox_LIBDIR} ${RCLLRefbox_IFACEDIR})
  set(RCLLRefbox_CFLAGS "-fPIC -pthread")
  set(RCLLRefbox_CFLAGS "${RCLLRefbox_CFLAGS} -DBASEDIR=\\\"${RCLLRefbox_BASEDIR}\\\" -RCLLRefbox_BASEDIR=\\\"${RCLLRefbox_COREDIR}\\\"")
  set(RCLLRefbox_CFLAGS "${RCLLRefbox_CFLAGS} -DBINDIR=\\\"${RCLLRefbox_BINDIR}\\\" -DLIBDIR=\\\"${RCLLRefbox_LIBDIR}\\\"")
  set(RCLLRefbox_CFLAGS "${RCLLRefbox_CFLAGS} -DPLUGINDIR=\\\"${RCLLRefbox_PLUGINDIR}\\\" -DIFACEDIR=\\\"${RCLLRefbox_IFACEDIR}\\\"")
  set(RCLLRefbox_CFLAGS "${RCLLRefbox_CFLAGS} -DCONFDIR=\\\"${RCLLRefbox_CONFDIR}\\\" -DUSERDIR=\\\"${RCLLRefbox_USERDIR}\\\"")
  set(RCLLRefbox_CFLAGS "${RCLLRefbox_CFLAGS} -DLOGDIR=\\\"${RCLLRefbox_LOGDIR}\\\" -DRESDIR=\\\"${RCLLRefbox_RESDIR}\\\"")
  set(RCLLRefbox_CFLAGS "${RCLLRefbox_CFLAGS} -DTMPDIR=\\\"${RCLLRefbox_TMPDIR}\\\"")

  foreach (COMPONENT ${RCLLRefbox_FIND_COMPONENTS})
    list(FIND RCLLRefbox_BASE_COMPONENTS ${COMPONENT} _BFOUND)
    if (_BFOUND GREATER -1)
      # It's a base component, just check for library
      find_library(RCLLRefbox_${COMPONENT} llsfrb${COMPONENT} HINTS ${RCLLRefbox_LIBDIR})
      if (RCLLRefbox_${COMPONENT})
	message(STATUS "  found RCLL refbox component ${COMPONENT}")
	list(APPEND RCLLRefbox_LIBRARIES llsfrb${COMPONENT})
      else()
	message(FATAL_ERROR "  RCLL refbox component ${COMPONENT} **NOT** found")
      endif()
    else()
      # It's a component that requires its own module
      find_package(RCLLRefbox_${COMPONENT} REQUIRED)
      if (RCLLRefbox_${COMPONENT}_FOUND)
	message(STATUS "  found RCLL refbox component ${COMPONENT}")
	list(APPEND RCLLRefbox_LIBRARIES ${RCLLRefbox_${COMPONENT}_LIBRARIES})
	list(APPEND RCLLRefbox_INCLUDE_DIRS ${RCLLRefbox_${COMPONENT}_INCLUDE_DIRS})
	list(APPEND RCLLRefbox_LIBRARY_DIRS ${RCLLRefbox_${COMPONENT}_LIBRARY_DIRS})
        set(RCLLRefbox_CFLAGS "${RCLLRefbox_CFLAGS} ${RCLLRefbox_${COMPONENT}_CFLAGS}")
	set(RCLLRefbox_LFLAGS "${RCLLRefbox_LFLAGS} ${RCLLRefbox_${COMPONENT}_LFLAGS}")
      else()
	message(FATAL_ERROR "  RCLL refbox component ${COMPONENT} **NOT** found")
      endif()
    endif()
  endforeach()

  foreach (LD ${RCLLRefbox_LIBRARY_DIRS})
    set(RCLLRefbox_LFLAGS "${RCLLRefbox_LFLAGS} -L${LD}")
  endforeach()

  string(STRIP "${RCLLRefbox_CFLAGS}" RCLLRefbox_CFLAGS)
  string(STRIP "${RCLLRefbox_LFLAGS}" RCLLRefbox_LFLAGS)
endif()
