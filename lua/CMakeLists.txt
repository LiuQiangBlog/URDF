cmake_minimum_required(VERSION 3.5)
project(lua VERSION 0.1.0 LANGUAGES C)

include_directories("${PROJECT_BINARY_DIR}")
#set (CMAKE_C_COMPILER "/std:c")
#set (CMAKE_CXX_COMPILER "/std:c++17")

include_directories("${PROJECT_BINARY_DIR}")

aux_source_directory(./src C_LIST)			# 将src下的所有源文件放在C_LIST变量中
list(REMOVE_ITEM C_LIST "./src/lua.c" "./src/luac.c")	# 从C_LIST中删除"./lua.c"和"./luac.c"

#file(GLOB luamain RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/src" "lua*.c") 	# 得到当前目录下所有以lua开头的源文件，即lua.c和luac.c
#message(STATUS ${luamain})
#message(STATUS "Compiler in use: ${CMAKE_C_COMPILER_ID}")
#foreach(mainfile IN LISTS ${luamain})
#    get_filename_component(mainname ${mainfile} NAME_WE)	# 得到文件名lua和luac，
#    add_executable(${mainname} ${mainfile} ${C_LIST})	# mainname为可执行文件的名字，mainfile C_LIST则是生成可执行文件的源文件。
#endforeach()

add_library (lualib ${C_LIST})

add_executable(lua ${CMAKE_CURRENT_SOURCE_DIR}/src/lua.c)
target_link_libraries(lua lualib)

add_executable(luac ${CMAKE_CURRENT_SOURCE_DIR}/src/luac.c)
target_link_libraries(luac lualib)


#ADD_DEFINITIONS(-DLUA_USE_LINUX -std=gnu99)

#add_library (lualib STATIC lapi.c lcode.c lctype.c ldebug.c ldo.c ldump.c lfunc.c lgc.c llex.c lmem.c lobject.c lopcodes.c lparser.c lstate.c lstring.c ltable.c ltm.c lundump.c lvm.c lzio.c lauxlib.c lbaselib.c lbitlib.c lcorolib.c ldblib.c liolib.c lmathlib.c loslib.c lstrlib.c ltablib.c loadlib.c linit.c lutf8lib.c)
#set_target_properties(lualib PROPERTIES OUTPUT_NAME "lua") # rename the library output name
#
#link_directories (${LUA_BINARY_DIR})
#add_executable (lua lua.c)
#target_link_libraries (lua lualib dl readline)
#if(UNIX)
#    target_link_libraries( lua m )
#endif()
#
#SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-E")
#
#link_directories (${LUA_BINARY_DIR})
#add_executable (luac luac.c)
#target_link_libraries (luac lualib dl readline)
#if(UNIX)
#    target_link_libraries( luac m )
#endif()
