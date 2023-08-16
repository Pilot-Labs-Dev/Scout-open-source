
set(HW_VERSION "3100000000")

set(SW_VERSION "2.0.0")

string(TIMESTAMP SW_BUILD_TIME "%Y-%m-%d %H:%M:%S")

execute_process(
  COMMAND git log -1 --pretty=format:%h
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  OUTPUT_VARIABLE SW_GIT_VERSION
  OUTPUT_STRIP_TRAILING_WHITESPACE
  )

set(VERSION_HEADER "${CMAKE_CURRENT_SOURCE_DIR}/include/roller_eye/version_info.h")
file(WRITE ${VERSION_HEADER} "#ifndef __ROLLER_EYE_VERSION_H__\r\n#define __ROLLER_EYE_VERSION_H__\r\n\r\n" )
file(APPEND ${VERSION_HEADER} "#define HW_VERSION \"${HW_VERSION}\"\r\n")
file(APPEND ${VERSION_HEADER} "#define SW_VERSION \"${SW_VERSION}\"\r\n")
file(APPEND ${VERSION_HEADER} "#define SW_BUILD_TIME \"${SW_BUILD_TIME}\"\r\n")
file(APPEND ${VERSION_HEADER} "#define SW_GIT_VERSION \"${SW_GIT_VERSION}\"\r\n")
file(APPEND ${VERSION_HEADER} "\r\n#endif\r\n")
