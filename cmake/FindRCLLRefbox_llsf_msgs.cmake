# - RCLLRefbox: check for llsf_msgs (fawkes-robotino only)
#
# Once done this will define
#
#  RCLLRefbox_llsf_msgs_FOUND - component was found
#  RCLLRefbox_llsf_msgs_CFLAGS - Flags to add to get tf support
#  RCLLRefbox_llsf_msgs_LIBRARIES - Libs required for tf support
#
# Copyright (c) 2015-2017  Tim Niemueller [www.niemueller.de]
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

if (NOT RCLLRefbox_FOUND)
  message(ERROR "Cannot check for component if RCLLRefbox not found")
  return()
endif()

find_package(RCLLRefbox_CPP11 REQUIRED)
find_package(Protobuf REQUIRED QUIET)
find_library(RCLLRefbox_llsf_msgs_LIBRARIES llsf_msgs HINTS "${RCLLRefbox_BASEDIR}/lib/protobuf")

if (RCLLRefbox_llsf_msgs_LIBRARIES)
  set(RCLLRefbox_llsf_msgs_FOUND TRUE)
  set(RCLLRefbox_llsf_msgs_CFLAGS "${RCLLRefbox_CPP11_CFLAGS} -DHAVE_LLSF_MSGS")
  set(RCLLRefbox_llsf_msgs_LIBRARIES llsf_msgs)
endif()
