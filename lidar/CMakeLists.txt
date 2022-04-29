cmake_minimum_required(VERSION 3.13.0)

# HEY!  Need to add additional files to the project?  (cpp, header, etc)
# Scroll down to line 30 or so to see where

# Linux folks: 
#   sudo apt install build-essential
#   sudo apt install libgl-dev
#   sudo apt install xorg-dev
#   sudo apt-get install freeglut3-dev

# HEY!  Want to change the name of the project/executable?  Do it on the next line
project(lidar LANGUAGES CXX C)

set(INCLUDE_SOUND TRUE)    # TRUE to include audio/sound support
set(INCLUDE_SERIAL TRUE)   # TRUE to include serial port support

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules/")

message("Before")
message(${CMAKE_INSTALL_PREFIX})

set(CMAKE_INSTALL_PREFIX ${CMAKE_BINARY_DIR})
message(${CMAKE_INSTALL_PREFIX})
message("After")



if(UNIX AND NOT APPLE)
    set(LINUX TRUE)
endif()

if(WIN32) 
set(ADD_EX_EXTRA WIN32)
else()
set(ADD_EX_EXTRA "")
endif()

add_executable(${PROJECT_NAME} ${ADD_EX_EXTRA}
    # Hey!  Add additional c, cpp, h files here:
    main.cpp
    CmdLink.h
    CmdLink.cpp


    third_party/rplidar_sdk/sdk/include/rplidar.h
    third_party/rplidar_sdk/sdk/include/rplidar_cmd.h
    third_party/rplidar_sdk/sdk/include/rplidar_driver.h
    third_party/rplidar_sdk/sdk/include/rplidar_protocol.h
    third_party/rplidar_sdk/sdk/include/rptypes.h
    third_party/rplidar_sdk/sdk/include/sl_crc.h
    third_party/rplidar_sdk/sdk/include/sl_lidar.h
    third_party/rplidar_sdk/sdk/include/sl_lidar_cmd.h
    third_party/rplidar_sdk/sdk/include/sl_lidar_driver.h
    third_party/rplidar_sdk/sdk/include/sl_lidar_driver_impl.h
    third_party/rplidar_sdk/sdk/include/sl_lidar_protocol.h
    third_party/rplidar_sdk/sdk/include/sl_types.h

    third_party/rplidar_sdk/sdk/src/sl_lidar_driver.cpp
    third_party/rplidar_sdk/sdk/src/hal/thread.cpp
    third_party/rplidar_sdk/sdk/src/sl_crc.cpp
    third_party/rplidar_sdk/sdk/src/sl_serial_channel.cpp
#    third_party/rplidar_sdk/sdk/src/sl_tcp_channel.cpp
#    third_party/rplidar_sdk/sdk/src/sl_udp_channel.cpp

    third_party/rplidar_sdk/sdk/src/arch/win32/net_serial.cpp
#    third_party/rplidar_sdk/sdk/src/arch/win32/net_socket.cpp
    third_party/rplidar_sdk/sdk/src/arch/win32/timer.cpp

)
#set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
#set(THREADS_PREFER_PTHREAD_FLAG TRUE)
#find_package(Threads REQUIRED)

#target_link_libraries(${PROJECT_NAME} Threads::Threads)

add_compile_definitions(CMAKE_PROJECT_NAME=${PROJECT_NAME})

add_compile_definitions(WITH_MINIAUDIO)

include_directories(
    third_party/nanovg
    third_party
    third_party/sago
    third_party/glfw/include
    graphics
    utility
    third_party/rplidar_sdk/sdk/include
    third_party/rplidar_sdk/sdk/src
    third_party/rplidar_sdk/sdk/src/arch/win32
    )

set(ASSETS_SOURCE_FOLDER "${CMAKE_CURRENT_SOURCE_DIR}/assets")

file(GLOB_RECURSE ASSETS "${ASSETS_SOURCE_FOLDER}/*")

# Filter values through regex
#   filter_regex({INCLUDE | EXCLUDE} <regex> <listname> [items...])
#   Element will included into result list if
#     INCLUDE is specified and it matches with regex or
#     EXCLUDE is specified and it doesn't match with regex.
# Example:
#   filter_regex(INCLUDE "(a|c)" LISTOUT a b c d) => a c
#   filter_regex(EXCLUDE "(a|c)" LISTOUT a b c d) => b d
function(filter_regex _action _regex _listname)
    # check an action
    if("${_action}" STREQUAL "INCLUDE")
        set(has_include TRUE)
    elseif("${_action}" STREQUAL "EXCLUDE")
        set(has_include FALSE)
    else()
        message(FATAL_ERROR "Incorrect value for ACTION: ${_action}")
    endif()

    set(${_listname})
    foreach(element ${ARGN})
        string(REGEX MATCH ${_regex} result ${element})
        if(result)
            if(has_include)
                list(APPEND ${_listname} ${element})
            endif()
        else()
            if(NOT has_include)
                list(APPEND ${_listname} ${element})
            endif()
        endif()
    endforeach()

    # put result in parent scope variable
    set(${_listname} ${${_listname}} PARENT_SCOPE)
endfunction()

filter_regex(EXCLUDE "\\.DS_Store" ASSETS ${ASSETS})

# add sources common to all platforms
target_sources(${PROJECT_NAME} PUBLIC
    ${ASSETS}
    third_party/glfw/src/context.c
    third_party/glfw/src/init.c
    third_party/glfw/src/input.c
    third_party/glfw/src/monitor.c
    third_party/glfw/src/vulkan.c
    third_party/glfw/src/window.c
    third_party/glfw/src/platform.c
    third_party/glfw/src/null_platform.h
    third_party/glfw/src/null_init.c
    third_party/glfw/src/null_window.c
    third_party/glfw/src/null_monitor.c
    third_party/glfw/src/null_joystick.c
    third_party/nanovg/nanovg.c

    third_party/whereami.c
    third_party/sago/platform_folders.cpp
    third_party/sago/platform_folders.h
    third_party/glfw/include/GLFW/glfw3.h
    third_party/glfw/include/GLFW/glfw3native.h
    third_party/whereami.h
    third_party/nanovg/fontstash.h
    third_party/nanovg/nanovg.h
    third_party/nanovg/nanovg_gl.h
    third_party/nanovg/nanovg_gl_utils.h
    third_party/nanovg/stb_image.h
    third_party/nanovg/stb_truetype.h

    graphics/graphics.cpp
    graphics/vec2d.cpp
    graphics/graphics.h
    graphics/vec2d.h

    utility/rand.h
    utility/rand.cpp
    utility/paths.h
    utility/paths.cpp
)

if(INCLUDE_SOUND)
    add_compile_definitions(INCLUDE_SOUND)
    include_directories(third_party/soloud/include)
    target_sources(${PROJECT_NAME}
        PUBLIC
        third_party/soloud/src/audiosource/wav/dr_impl.cpp
        third_party/soloud/src/audiosource/wav/soloud_wav.cpp
        third_party/soloud/src/audiosource/wav/soloud_wavstream.cpp
        third_party/soloud/src/audiosource/wav/stb_vorbis.c
        third_party/soloud/src/backend/miniaudio/soloud_miniaudio.cpp
        third_party/soloud/src/core/soloud.cpp
        third_party/soloud/src/core/soloud_audiosource.cpp
        third_party/soloud/src/core/soloud_bus.cpp
        third_party/soloud/src/core/soloud_core_3d.cpp
        third_party/soloud/src/core/soloud_core_basicops.cpp
        third_party/soloud/src/core/soloud_core_faderops.cpp
        third_party/soloud/src/core/soloud_core_filterops.cpp
        third_party/soloud/src/core/soloud_core_getters.cpp
        third_party/soloud/src/core/soloud_core_setters.cpp
        third_party/soloud/src/core/soloud_core_voicegroup.cpp
        third_party/soloud/src/core/soloud_core_voiceops.cpp
        third_party/soloud/src/core/soloud_fader.cpp
        third_party/soloud/src/core/soloud_fft.cpp
        third_party/soloud/src/core/soloud_fft_lut.cpp
        third_party/soloud/src/core/soloud_file.cpp
        third_party/soloud/src/core/soloud_filter.cpp
        third_party/soloud/src/core/soloud_misc.cpp
        third_party/soloud/src/core/soloud_queue.cpp
        third_party/soloud/src/core/soloud_thread.cpp
        third_party/soloud/include/soloud.h
        third_party/soloud/include/soloud_audiosource.h
        third_party/soloud/include/soloud_bassboostfilter.h
        third_party/soloud/include/soloud_biquadresonantfilter.h
        third_party/soloud/include/soloud_bus.h
        third_party/soloud/include/soloud_c.h
        third_party/soloud/include/soloud_dcremovalfilter.h
        third_party/soloud/include/soloud_echofilter.h
        third_party/soloud/include/soloud_error.h
        third_party/soloud/include/soloud_fader.h
        third_party/soloud/include/soloud_fft.h
        third_party/soloud/include/soloud_fftfilter.h
        third_party/soloud/include/soloud_file.h
        third_party/soloud/include/soloud_file_hack_off.h
        third_party/soloud/include/soloud_file_hack_on.h
        third_party/soloud/include/soloud_filter.h
        third_party/soloud/include/soloud_flangerfilter.h
        third_party/soloud/include/soloud_freeverbfilter.h
        third_party/soloud/include/soloud_internal.h
        third_party/soloud/include/soloud_lofifilter.h
        third_party/soloud/include/soloud_misc.h
        third_party/soloud/include/soloud_monotone.h
        third_party/soloud/include/soloud_noise.h
        third_party/soloud/include/soloud_openmpt.h
        third_party/soloud/include/soloud_queue.h
        third_party/soloud/include/soloud_robotizefilter.h
        third_party/soloud/include/soloud_sfxr.h
        third_party/soloud/include/soloud_speech.h
        third_party/soloud/include/soloud_tedsid.h
        third_party/soloud/include/soloud_thread.h
        third_party/soloud/include/soloud_vic.h
        third_party/soloud/include/soloud_vizsn.h
        third_party/soloud/include/soloud_wav.h
        third_party/soloud/include/soloud_waveshaperfilter.h
        third_party/soloud/include/soloud_wavstream.h
        third_party/soloud/src/audiosource/wav/dr_flac.h
        third_party/soloud/src/audiosource/wav/dr_mp3.h
        third_party/soloud/src/audiosource/wav/dr_wav.h
        third_party/soloud/src/audiosource/wav/stb_vorbis.h
        third_party/soloud/src/backend/miniaudio/miniaudio.h
        )
endif()



if (WIN32)
    # add sources specific to Windows
target_sources(${PROJECT_NAME}
    PUBLIC
    third_party/glfw/src/win32_init.c
    third_party/glfw/src/win32_joystick.c
    third_party/glfw/src/win32_monitor.c
    third_party/glfw/src/win32_time.c
    third_party/glfw/src/win32_thread.c
    third_party/glfw/src/win32_window.c
    third_party/glfw/src/wgl_context.c
    third_party/glfw/src/egl_context.c
    third_party/glfw/src/osmesa_context.c
    third_party/glfw/src/win32_module.c
    third_party/glew.c
    third_party/glfw/src/win32_platform.h
    third_party/glfw/src/win32_joystick.h
    third_party/GL/eglew.h
    third_party/GL/glxew.h
    third_party/GL/wglew.h
    third_party/GL/glew.h
)
target_link_libraries(${PROJECT_NAME} gdi32 opengl32)
add_compile_definitions(_GLFW_WIN32 NANOVG_GLEW GLEW_STATIC PROVIDE_WINMAIN )
set_target_properties(${PROJECT_NAME} PROPERTIES
    WIN32_EXECUTABLE TRUE
)
endif()




if (APPLE)
    # add sources specific to macos
    target_sources(${PROJECT_NAME}
        PUBLIC
        third_party/glfw/src/cocoa_init.m
        third_party/glfw/src/cocoa_joystick.m
        third_party/glfw/src/cocoa_monitor.m
        third_party/glfw/src/cocoa_window.m
        third_party/glfw/src/cocoa_time.c
        third_party/glfw/src/posix_thread.c
        third_party/glfw/src/nsgl_context.m
        third_party/glfw/src/egl_context.c
        third_party/glfw/src/osmesa_context.c
        third_party/glfw/src/posix_module.c
        third_party/glfw/src/cocoa_platform.h
        third_party/glfw/src/cocoa_joystick.h
        third_party/glfw/src/posix_thread.h
    )
target_link_libraries(${PROJECT_NAME}
    "-framework Cocoa"
    "-framework IOKit"
    "-framework OpenGL")
add_compile_definitions(_GLFW_COCOA GL_SILENCE_DEPRECATION)

message("Switch to METAL on Mac at some point???")
set_target_properties(${PROJECT_NAME} PROPERTIES
  MACOSX_BUNDLE TRUE
  MACOSX_FRAMEWORK_IDENTIFIER org.mssm.${PROJECT_NAME}
  MACOSX_BUNDLE_BUNDLE_NAME ${PROJECT_NAME}
  #  MACOSX_BUNDLE_SHORT_VERSION_STRING ${VERSION}
)
endif()



if (LINUX)
    # add sources specific to macos
    target_sources(${PROJECT_NAME}
        PUBLIC
       third_party/glfw/src/x11_init.c
       third_party/glfw/src/x11_monitor.c
       third_party/glfw/src/x11_window.c
       third_party/glfw/src/xkb_unicode.c
       third_party/glfw/src/posix_time.c
       third_party/glfw/src/posix_thread.c
       third_party/glfw/src/glx_context.c
       third_party/glfw/src/egl_context.c
       third_party/glfw/src/osmesa_context.c
       third_party/glfw/src/linux_joystick.c
       third_party/glew.c
       third_party/glfw/src/posix_module.c
       third_party/glfw/src/posix_poll.c
       third_party/glfw/src/posix_poll.h

       third_party/glfw/src/linux_joystick.h
       third_party/glfw/src/x11_platform.h
       third_party/glfw/src/xkb_unicode.h
       third_party/glfw/src/posix_time.h
       third_party/glfw/src/posix_thread.h
       third_party/GL/eglew.h
       third_party/GL/glxew.h
       third_party/GL/wglew.h
       third_party/GL/glew.h
    )

    find_package(OpenGL REQUIRED)
    find_package(GLUT REQUIRED)
    find_package(X11 REQUIRED)
    # x11 xrandr xi xxf86vm glew glfw3

    target_link_libraries(${PROJECT_NAME} OpenGL::GL GLUT::GLUT ${CMAKE_DL_LIBS} ${X11_LIBRARIES} pthread)

    
    add_compile_definitions(_GLFW_X11 NANOVG_GLEW GLEW_STATIC)
endif()



if(INCLUDE_SERIAL)
    add_compile_definitions(INCLUDE_SERIAL SP_PRIV=)
    include_directories(third_party/libserialport comm)
    target_sources(${PROJECT_NAME}
        PUBLIC
        third_party/libserialport/libserialport.h
        third_party/libserialport/libserialport_internal.h
        third_party/libserialport/serialport.c
        comm/serialport.h
        comm/serialport.cpp
    )

if(WIN32)
    target_sources(${PROJECT_NAME}
        PUBLIC
        third_party/libserialport/windows.c
        third_party/libserialport/timing.c
    )
    target_link_libraries(${PROJECT_NAME} gdi32 opengl32 Cfgmgr32 SetupAPI)

elseif(APPLE)
    target_sources(${PROJECT_NAME}
        PUBLIC
        third_party/libserialport/macosx.c
        third_party/libserialport/timing.c
    )

elseif(LINUX)
    target_sources(${PROJECT_NAME}
        PUBLIC
        third_party/libserialport/linux.c
        third_party/libserialport/linux_termios.c
        third_party/libserialport/linux_termios.h
#        third_party/libserialport/freebsd.c
    )
endif()
endif()


if(WIN32) # Check if we are on Windows
    if(MSVC) # Check if we are using the Visual Studio compiler
        set_target_properties(${PROJECT_NAME} PROPERTIES WIN32_EXECUTABLE YES LINK_FLAGS "/ENTRY:mainCRTStartup" )
    elseif(CMAKE_COMPILER_IS_GNUCXX) #
        SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mwindows") # Not tested
    else()
        message(SEND_ERROR "You are using an unsupported Windows compiler! (Not MSVC or GCC)")
    endif(MSVC)
elseif(APPLE)
    set_target_properties(${PROJECT_NAME} PROPERTIES MACOSX_BUNDLE YES )
elseif(UNIX)
    # Nothing special required
else()
    message(SEND_ERROR "You are on an unsupported platform! (Not Win32, Mac OS X or Unix)")
endif(WIN32)


#set(ASSETS ${CMAKE_SOURCE_DIR}/assets/Roboto-Bold.ttf)
#target_sources(${PROJECT_NAME} PRIVATE ${ASSETS})

#set_target_properties(${PROJECT_NAME} PROPERTIES
#  MACOSX_BUNDLE ON
#  MACOSX_BUNDLE_BUNDLE_NAME ${PROJECT_NAME}
##  MACOSX_BUNDLE_SHORT_VERSION_STRING ${VERSION}
#  RESOURCE "${ASSETS}"
#)

#message(${CMAKE_COMMAND})
#message(${CMAKE_CURRENT_BINARY_DIR})

set(ASSET_DEST_FILES "")

# loop over assets and make sure they get copied to the executable location
foreach (FILE ${ASSETS})

    # Get the relative path from the data-folder to the particular file.
    file(RELATIVE_PATH NEW_FILE "${ASSETS_SOURCE_FOLDER}" ${FILE})

    # Get the relative path to the file.
    get_filename_component(NEW_FILE_PATH ${NEW_FILE} DIRECTORY)

    # MAC:  Set it's location inside the app package (under Resources).
    set_property(SOURCE ${FILE} PROPERTY MACOSX_PACKAGE_LOCATION "Resources/${NEW_FILE_PATH}")

#    message("Setting property")

    if(NOT APPLE)
        set(OUTF "${CMAKE_CURRENT_BINARY_DIR}/assets/${NEW_FILE}")

        list(APPEND ASSET_DEST_FILES ${OUTF})

        get_filename_component(OUTF_DIR "${OUTF}" DIRECTORY)

#        message("Adding custom command")
        add_custom_command(
            OUTPUT "${OUTF}"
            COMMAND ${CMAKE_COMMAND} -E make_directory "${OUTF_DIR}"
            COMMAND ${CMAKE_COMMAND} -E copy_if_different ${FILE} ${OUTF}
            COMMENT "Copying ${FILE} to ${OUTF}"
            DEPENDS ${FILE}
            VERBATIM
        )
    endif()

    # Make sure it shows up in the IDE Assets folder
    source_group("Assets/${NEW_FILE_PATH}" FILES "${FILE}")

endforeach ()

if(NOT APPLE)
    add_custom_target(asset_dest_target
        DEPENDS
            ${ASSET_DEST_FILES}
    )

    add_dependencies(${PROJECT_NAME} asset_dest_target)
endif()

install(TARGETS ${PROJECT_NAME} DESTINATION .)

if (APPLE)
set(APPS "\${CMAKE_INSTALL_PREFIX}/${PROJECT_NAME}.app")
set(CPACK_GENERATOR "DragNDrop")
else()
set(APPS "\${CMAKE_INSTALL_PREFIX}/${PROJECT_NAME}.exe")
endif()

set(DIRS ${CMAKE_BINARY_DIR})

message(${CMAKE_BINARY_DIR})

message(${CMAKE_COMMAND})

#get_cmake_property(_variableNames VARIABLES)
#foreach (_variableName ${_variableNames})
#    message(STATUS "${_variableName}=${${_variableName}}")
#endforeach()

install(DIRECTORY ${CMAKE_SOURCE_DIR}/assets/ DESTINATION assets)

install(CODE "include(BundleUtilities)
fixup_bundle(\"${APPS}\" \"\" \"${DIRS}\")")




#set(MINGW_PATH ${CMAKE_PREFIX_PATH})
#message(${MINGW_PATH})
#set(CMAKE_INSTALL_SYSTEM_RUNTIME_DESTINATION .)


if(UNIX AND NOT APPLE)
    get_filename_component(GCC_PATH ${CMAKE_CXX_COMPILER} DIRECTORY)
    find_library(LIBGCC NAMES libgcc_s.so.1 HINTS ${GCC_PATH}/../lib64)
    get_filename_component(LIBGCC_DIR ${LIBGCC} DIRECTORY)
    file(GLOB LIBGCC_ALL ${LIBGCC_DIR}/libgcc*.so.1)
    find_library(STDCPP NAMES libstdc++.so.6 HINTS ${GCC_PATH}/../lib64)
    file(GLOB STDCPP_ALL ${STDCPP}*)
    set(CMAKE_INSTALL_SYSTEM_RUNTIME_LIBS
    ${LIBGCC_ALL}
    ${STDCPP_ALL})
endif()




if(WIN32 AND NOT MSVC)
    set(CMAKE_FIND_LIBRARY_SUFFIXES .dll) # important!
    GET_FILENAME_COMPONENT(GCC_PATH ${CMAKE_CXX_COMPILER} DIRECTORY)

#    foreach(RUNTIME
#        libstdc++-6.dll
#        libgcc_s_seh-1.dll
#        libwinpthread-1.dll
#        )
#        message(STATUS "Checking for ${RUNTIME}")
#        find_library(${RUNTIME}_LIB NAMES ${RUNTIME})
#        message(${${RUNTIME}_LIB})
#        message("Hey")
#        message(${RUNTIME}_LIB-FOUND)
#        if(${RUNTIME}_LIB-FOUND)
#            message(STATUS "runtime found: ${${RUNTIME}_LIB}")
#            list(APPEND CMAKE_INSTALL_SYSTEM_RUNTIME_LIBS ${${RUNTIME}_LIB})
#        else()
#            message(STATUS "${RUNTIME} not found. Hope nothing breaks!")
#        endif()
#    endforeach()


    find_library (LIBGCC NAMES libgcc_s_seh-1 HINTS ${GCC_PATH})
    if (LIBGCC)
      message (STATUS "Found libgcc        at ${LIBGCC}")
      list(APPEND CMAKE_INSTALL_SYSTEM_RUNTIME_LIBS ${LIBGCC})
    else ()
      message (SEND_ERROR "Could not find libgcc!")
    endif ()

    find_library (LIBSTDCPP libstdc++-6 HINTS ${GCC_PATH})
    if (LIBSTDCPP)
      message (STATUS "Found libstdc++     at ${LIBSTDCPP}")
      list(APPEND CMAKE_INSTALL_SYSTEM_RUNTIME_LIBS ${LIBSTDCPP})
    else ()
      message (SEND_ERROR "Could not find libstdc++!")
    endif ()

    find_library (LIBWINPTHREAD libwinpthread-1 HINTS ${GCC_PATH})
    if (LIBWINPTHREAD)
      message (STATUS "Found libwinpthread at ${LIBWINPTHREAD}")
      list(APPEND CMAKE_INSTALL_SYSTEM_RUNTIME_LIBS ${LIBWINPTHREAD})
    else ()
      message (SEND_ERROR "Could not find libwinpthread!")
    endif ()

endif()

message(STATUS "WHAT")
message(STATUS ${CMAKE_INSTALL_SYSTEM_RUNTIME_LIBS})
message(STATUS ${GCC_PATH})

include( InstallRequiredSystemLibraries )


include(cpack)
