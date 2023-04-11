cmake_minimum_required(VERSION 3.16)

# DYNAMORIO_HOME=/scarab/src/build/opt/deps/dynamorio

project(simulate4)

# link_directories($ENV{DYNAMORIO_HOME}/tools/lib64/release)
link_directories($ENV{DYNAMORIO_HOME}/clients/lib64/release)
link_directories($ENV{DYNAMORIO_HOME}/ext/lib64/release)
link_directories(/scarab/src/deps/dynamorio/ext/drsyms/libelftc/lib64)

if(COMMAND cmake_policy)
	cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

add_executable(simulate4 simulate4.cpp)

target_include_directories(simulate4 PRIVATE "/SCOTSv0.2_Copy/src/"
"/SCOTSv0.2_Copy/utils/")


# configure_file(controller.scs controller.scs COPYONLY)

find_package(DynamoRIO)
if (NOT DynamoRIO_FOUND)
  message(FATAL_ERROR "DynamoRIO package required to build")
endif(NOT DynamoRIO_FOUND)

find_package(ZLIB)
if (ZLIB_FOUND)
  message(STATUS "Found zlib")
endif ()

find_library(libsnappy snappy)
if (libsnappy)
  message(STATUS "Found snappy: ${libsnappy}")
endif ()

find_library(liblz4 lz4)
if (liblz4)
  message(STATUS "Found liblz4: ${liblz4}")
endif ()

configure_DynamoRIO_static(simulate4)

use_DynamoRIO_static_client(simulate4 drmemtrace_static)

if (ZLIB_FOUND)
  target_link_libraries(simulate4 ${ZLIB_LIBRARIES})
endif ()
if (libsnappy)
  target_link_libraries(simulate4 snappy)
endif ()
if (liblz4)
  target_link_libraries(simulate4 lz4)
endif ()

use_DynamoRIO_static_client(simulate4 drmgr_static)
use_DynamoRIO_static_client(simulate4 drreg_static)
use_DynamoRIO_static_client(simulate4 drbbdup_static)
use_DynamoRIO_static_client(simulate4 drutil_static)
use_DynamoRIO_static_client(simulate4 drx_static)
use_DynamoRIO_static_client(simulate4 drwrap_static)
use_DynamoRIO_static_client(simulate4 drsyms_static)
# use_DynamoRIO_static_client(simulate4 drpttracer_static)
use_DynamoRIO_static_client(simulate4 drstatecmp_static)
use_DynamoRIO_static_client(simulate4 drcovlib_static)
