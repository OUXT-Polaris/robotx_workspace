cmake_minimum_required(VERSION 3.5)
project(lua_vendor)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

macro(build_lua)
  set(cmake_commands)
  set(cmake_configure_args -Wno-dev;-DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_install)
  include(ExternalProject)
  set(lua_source_path ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_install/src/lua-ext)
  set(lua_cmake_file_path ${lua_source_path}/CMakeLists.txt)
  externalproject_add(lua-ext
    PREFIX            ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_install
    INSTALL_DIR       ${CMAKE_INSTALL_PREFIX}
    URL               http://www.lua.org/ftp/lua-5.3.2.tar.gz
    URL_HASH          SHA1=7a47adef554fdca7d0c5536148de34579134a973
    PATCH_COMMAND
      COMMAND echo cmake_minimum_required( VERSION 3.2 ) > ${lua_cmake_file_path}
      COMMAND echo file( GLOB_RECURSE lua_sources src/*.c ) >> ${lua_cmake_file_path}
      COMMAND echo list( REMOVE_ITEM lua_sources $\{CMAKE_CURRENT_SOURCE_DIR}/src/lua.c $\{CMAKE_CURRENT_SOURCE_DIR}/src/luac.c ) >> ${lua_cmake_file_path}
      COMMAND echo project( lua_vendor ) >> ${lua_cmake_file_path}
      COMMAND echo add_library( lua_vendor STATIC $\{lua_sources} ) >> ${lua_cmake_file_path}
      COMMAND echo install( TARGETS lua_vendor DESTINATION lib ) >> ${lua_cmake_file_path}
      COMMAND echo install( DIRECTORY src/ DESTINATION include FILES_MATCHING PATTERN "lua*.h*" ) >> ${lua_cmake_file_path}
      COMMAND echo install( DIRECTORY src/ DESTINATION include FILES_MATCHING PATTERN "lauxlib.h" ) >> ${lua_cmake_file_path}
    CMAKE_ARGS
      ${cmake_configure_args}
  )
  install(
    DIRECTORY
      ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_install/
    DESTINATION
      ${CMAKE_INSTALL_PREFIX}
  )
endmacro()

build_lua()
ament_export_include_directories(include)
ament_export_libraries(lua_vendor)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()