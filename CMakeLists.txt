
cmake_minimum_required(VERSION 3.5)
project(realtime-network-av)

option(USE_OMX "Build programs that use the OMX library on Raspberry Pi" OFF)
option(USE_CYTHON "Build programs that requite cython" ON)

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

INCLUDE(VersionMacros)

if (WIN32)
  set(Extra_Libraries "ws2_32")
  set(MSYS_DIR "c:/msys64/mingw64")
else()
  set(Extra_Libraries "pthread")
endif()

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 -DNDEBUG")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_hypot=hypot -O3 -DNDEBUG -DMS_WIN64 -DBOOST_LOG_DYN_LINK")

# We have submodule, so we need git to update the submodule
find_package(Git QUIET)

# Update submodules as needed
if(GIT_FOUND AND EXISTS "${PROJECT_SOURCE_DIR}/.git")
  option(GIT_SUBMODULE "Check submodules during build" OFF)
  if(GIT_SUBMODULE)
    message(STATUS "Submodule update")
    execute_process(COMMAND ${GIT_EXECUTABLE} submodule update --init --recursive
      WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
      RESULT_VARIABLE GIT_SUBMOD_RESULT)
    if(NOT GIT_SUBMOD_RESULT EQUAL "0")
      message(FATAL_ERROR "git submodule update --init failed with ${GIT_SUBMOD_RESULT}, please checkout submodules")
    endif()
  endif()
endif()

# Ensure the rtaudio directory was checked out successfully
if(NOT EXISTS "${PROJECT_SOURCE_DIR}/modules/rtaudio/CMakeLists.txt")
  message(FATAL_ERROR "The rtaudio submodule was not downloaded! GIT_SUBMODULE was turned off or failed. Please update submodules and try again.")
endif()

# Ensure the mavlink directory was checked out successfully
if(NOT EXISTS "${PROJECT_SOURCE_DIR}/modules/mavlink/protocol.h")
  message(FATAL_ERROR "The mavlink submodule was not downloaded! GIT_SUBMODULE was turned off or failed. Please update submodules and try again.")
endif()

# Ensure the mavlink directory was checked out successfully
if(NOT EXISTS "${PROJECT_SOURCE_DIR}/modules/dcadec/dcadec.c")
  message(FATAL_ERROR "The dcadec submodule was not downloaded! GIT_SUBMODULE was turned off or failed. Please update submodules and try again.")
endif()

# Add the local include directory to the include path
include_directories(${PROJECT_SOURCE_DIR}/include)

if (USE_OMX)
  include_directories(/opt/vc/include /opt/vc/include/interface/vcos/pthreads /opt/vc/include/interface/vmcs_host/linux)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DSTANDALONE -D__STDC_CONSTANT_MACROS -D__STDC_LIMIT_MACROS -DTARGET_POSIX -D_LINUX -DPIC -D_REENTRANT -D_LARGEFILE64_SOURCE -D_FILE_OFFSET_BITS=64 -U_FORTIFY_SOURCE -DHAVE_LIBOPENMAX=2 -DOMX -DOMX_SKIP64BIT -ftree-vectorize -pipe -DUSE_EXTERNAL_OMX -DHAVE_LIBBCM_HOST -DUSE_EXTERNAL_LIBBCM_HOST -DUSE_VCHIQ_ARM")
  link_directories("/opt/vc/lib")
  add_executable(rpi-camera-server src/rpi-camera-server.c)
  target_link_libraries(rpi-camera-server openmaxil bcm_host vcos pthread)
  install(TARGETS rpi-camera-server DESTINATION bin COMPONENT applications)
endif (USE_OMX)

# Add rtaudio to the list of directories to build
add_subdirectory(modules/rtaudio)

# Add srt to the list of directories to build
add_subdirectory(modules/srt)

# Add the python directory to the list of directories to build
add_subdirectory(python)

# Add the rtaudio directory to the paths
include_directories("${PROJECT_SOURCE_DIR}/modules/rtaudio")
link_directories("${PROJECT_SOURCE_DIR}/modules/rtaudio")

# Find the boost libraries
find_package(Boost REQUIRED COMPONENTS
  program_options
  system
  filesystem)
include_directories(${Boost_INCLUDE_DIR})

# Build the udp_audio server
add_executable(udp_audio_player src/udp_audio_player.cc)
target_link_libraries(udp_audio_player rtaudio ${Extra_Libraries} ${Boost_LIBRARIES})
install(TARGETS udp_audio_player DESTINATION bin COMPONENT applications)

add_subdirectory(modules/dcadec)

# To find and use ffmpeg avcodec library
find_package(FFmpeg)
include_directories(${AVCODEC_INCLUDE_DIR})
find_path(DCADEC_INCLUDE_DIR libdcadec/common.h PATHS ${PROJECT_SOURCE_DIR}/modules/dcadec)
find_library(DCADEC_LIBRARY dcacec PATHS ${PROJECT_SOURCE_DIR}/modules/dcadec)
include_directories(${DCADEC_INCLUDE_DIR})

find_package(SDL2)
find_library(SDL2_TTF_LIBRARY SDL2_ttf)
find_library(SDL2_IMAGE_LIBRARY SDL2_image)
find_library(SDL2_GFX_LIBRARY SDL2_gfx)

# Add the mavlink directory to the include path
include_directories(${PROJECT_SOURCE_DIR}/modules/mavlink/common)

include_directories(${SDL2_INCLUDE_DIRS} ${SRT_SOURCE_DIR}/srtcore ${SRT_SOURCE_DIR}/common ${SRT_BINARY_DIR} ${PROJECT_BINARY_DIR}/sdl/include)
link_directories(${PROJECT_BINARY_DIR})
add_executable(video_stream_player
  src/video_stream_player.cc
  src/telemetry.cc
  src/texture.cc
  src/ffmpeg_decoder.cc
  src/sdl_osd.cc
  src/sdl_render_window.cc
  src/transmitter.cc
  src/fec.c
  src/fec.cc
  src/raw_socket.cc
  src/radiotap.c)
target_link_libraries(video_stream_player ${AVCODEC_LIBRARIES} ${AVFORMAT_LIBRARIES} ${AVUTIL_LIBRARIES} ${SWSCALE_LIBRARIES} ${SDL2_LIBRARIES} ${SDL2_TTF_LIBRARY} ${SDL2_IMAGE_LIBRARY} ${SDL2_GFX_LIBRARY} ${Boost_LIBRARIES} ${Extra_Libraries} srt_shared ssl crypto pcap)
install(TARGETS video_stream_player DESTINATION bin COMPONENT applications)

find_library(V4L_LIBRARY v4l2)
if (V4L_LIBRARY)
  add_executable(v4l_h264_server
    src/v4l_h264_server.cc)
  target_link_libraries(v4l_h264_server ${V4L_LIBRARY} ${Boost_LIBRARIES} ${Extra_Libraries})
  install(TARGETS v4l_h264_server DESTINATION bin COMPONENT applications)
endif (V4L_LIBRARY)

find_package(ALSA)
if (ALSA_FOUND)
  include_directories(${ALSA_INCLUDE_DIRS})
  add_executable(spdif_to_udp src/spdif_to_udp.cc)
  link_directories(modules/dcadec)
  target_link_libraries(spdif_to_udp dcadec asound ${Boost_LIBRARIES})
  install(TARGETS spdif_to_udp DESTINATION bin COMPONENT applications)
endif ()

find_library(OPENMAXIL_LIBRARY openmaxil HINT /opt/vc/lib)
if (OPENMAXIL_LIBRARY)
  find_library(BRCMGLES_LIBRARY brcmGLESv2 HINT /opt/vc/lib)
  find_library(BRCMEGL_LIBRARY brcmEGL HINT /opt/vc/lib)
  find_library(OPENMAXIL_LIBRARY openmaxil  HINT /opt/vc/lib)
  find_library(BCM_HOST_LIBRARY bcm_host HINT /opt/vc/lib)
  find_library(VCOS_LIBRARY vcos HINT /opt/vc/lib)
  find_library(VCHIQ_LIBRARY vchiq_arm  HINT /opt/vc/lib)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 -DNDEBUG -DSTANDALONE -D__STDC_CONSTANT_MACROS -D__STDC_LIMIT_MACROS -DTARGET_POSIX -D_LINUX -fPIC -DPIC -D_REENTRANT -D_LARGEFILE64_SOURCE -D_FILE_OFFSET_BITS=64 -U_FORTIFY_SOURCE -Wall -DHAVE_LIBOPENMAX=2 -DOMX -DOMX_SKIP64BIT -ftree-vectorize -pipe -DUSE_EXTERNAL_OMX -DHAVE_LIBBCM_HOST -DUSE_EXTERNAL_LIBBCM_HOST -DUSE_VCHIQ_ARM -Wno-psabi")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CMAKE_C_FLAGS}")
  include_directories(/opt/vc/include /opt/vc/src/hello_pi/libs/ilclient/)
  link_directories(/opt/vc/lib)
  add_executable(rpi_video_player src/rpi_video_player.cc src/ilclient.c src/ilcore.c)
  target_link_libraries(rpi_video_player ${OPENMAXIL_LIBRARY} ${BRCMGLES_LIBRARY} ${BRCMEGL_LIBRARY} ${OPENMAXIL_LIBRARY} ${BCM_HOST_LIBRARY} ${VCOS_LIBRARY} ${VCHIQ_LIBRARY} ${Extra_Libraries})
  install(TARGETS rpi_video_player DESTINATION bin COMPONENT applications)
endif ()

add_executable(raw_wifi_bridge src/raw_wifi_bridge.cc src/raw_socket.cc src/fec.c src/fec.cc src/radiotap.c)
install(TARGETS raw_wifi_bridge DESTINATION bin COMPONENT applications)
target_link_libraries(raw_wifi_bridge ${Boost_LIBRARIES} pcap pthread)

add_executable(raw_wifi_receiver src/raw_wifi_receiver.cc src/raw_socket.cc src/radiotap.c src/fec.c src/fec.cc)
install(TARGETS raw_wifi_receiver DESTINATION bin COMPONENT applications)
target_link_libraries(raw_wifi_receiver ${Boost_LIBRARIES} pcap pthread)

add_executable(csrf_receiver src/csrf_receiver.cc)
install(TARGETS csrf_receiver DESTINATION bin COMPONENT applications)
target_link_libraries(csrf_receiver ${Boost_LIBRARIES} pthread)

# Install the radiotap python directory
install(FILES
  ${PROJECT_SOURCE_DIR}/modules/python-radiotap/radiotap/__init__.py
  ${PROJECT_SOURCE_DIR}/modules/python-radiotap/radiotap/radiotap.py
  ${PROJECT_SOURCE_DIR}/modules/python-radiotap/radiotap/vht.py
  DESTINATION python/python-radiotap)

# Install the icons
install(FILES "${PROJECT_SOURCE_DIR}/icons/attitude-background.png" DESTINATION icons COMPONENT icons)
install(FILES "${PROJECT_SOURCE_DIR}/icons/attitude-foreground.png" DESTINATION icons COMPONENT icons)
install(FILES "${PROJECT_SOURCE_DIR}/icons/attitude-ground.png" DESTINATION icons COMPONENT icons)
install(FILES "${PROJECT_SOURCE_DIR}/icons/attitude-ring.png" DESTINATION icons COMPONENT icons)
install(FILES "${PROJECT_SOURCE_DIR}/icons/home-arrow.png" DESTINATION icons COMPONENT icons)
install(FILES "${PROJECT_SOURCE_DIR}/icons/north-arrow.png" DESTINATION icons COMPONENT icons)

# Install the font files
install(FILES "${PROJECT_SOURCE_DIR}/fonts/Cousine-Bold.ttf" DESTINATION fonts COMPONENT fonts)
install(FILES "${PROJECT_SOURCE_DIR}/fonts/Cousine-BoldItalic.ttf" DESTINATION fonts COMPONENT fonts)
install(FILES "${PROJECT_SOURCE_DIR}/fonts/Cousine-Italic.ttf" DESTINATION fonts COMPONENT fonts)
install(FILES "${PROJECT_SOURCE_DIR}/fonts/Cousine-Regular.ttf" DESTINATION fonts COMPONENT fonts)

# Install the configuration files
install(FILES "${PROJECT_SOURCE_DIR}/conf/wifi_bridge_ground.json" DESTINATION conf COMPONENT configuratios)

# Create a windows installer
if (WIN32)
 
  # Install the dependent dlls to the bin directory
  install(FILES "${MSYS_DIR}/bin/libwinpthread-1.dll" DESTINATION bin COMPONENT dlls)
  install(FILES "${MSYS_DIR}/bin/libgcc_s_seh-1.dll" DESTINATION bin COMPONENT dlls)
  install(FILES "${MSYS_DIR}/bin/avcodec-58.dll" DESTINATION bin COMPONENT dlls)
  install(FILES "${MSYS_DIR}/bin/libstdc++-6.dll" DESTINATION bin COMPONENT dlls)
  install(FILES "${MSYS_DIR}/bin/avformat-58.dll" DESTINATION bin COMPONENT dlls)
  install(FILES "${MSYS_DIR}/bin/avutil-56.dll" DESTINATION bin COMPONENT dlls)
  install(FILES "${MSYS_DIR}/bin/libboost_program_options-mt.dll" DESTINATION bin COMPONENT dlls)
  install(FILES "${MSYS_DIR}/bin/SDL2.dll" DESTINATION bin COMPONENT dlls)
  install(FILES "${MSYS_DIR}/bin/libSDL2_gfx-1-0-0.dll" DESTINATION bin COMPONENT dlls)
  install(FILES "${MSYS_DIR}/bin/SDL2_image.dll" DESTINATION bin COMPONENT dlls)
  install(FILES "${MSYS_DIR}/bin/SDL2_ttf.dll" DESTINATION bin COMPONENT dlls)
  install(FILES "${MSYS_DIR}/bin/swscale-5.dll" DESTINATION bin COMPONENT dlls)
  install(FILES "${MSYS_DIR}/bin/libbz2-1.dll" DESTINATION bin COMPONENT dlls)
  install(FILES "${MSYS_DIR}/bin/libgnutls-30.dll" DESTINATION bin COMPONENT dlls)
  install(FILES "${MSYS_DIR}/bin/libcelt0-2.dll" DESTINATION bin COMPONENT dlls)
  install(FILES "${MSYS_DIR}/bin/libgsm.dll" DESTINATION bin COMPONENT dlls)
  install(FILES "${MSYS_DIR}/bin/libiconv-2.dll" DESTINATION bin COMPONENT dlls)
  install(FILES "${MSYS_DIR}/bin/libmfx-1.dll" DESTINATION bin COMPONENT dlls)
  install(FILES "${MSYS_DIR}/bin/liblzma-5.dll" DESTINATION bin COMPONENT dlls)
  install(FILES "${MSYS_DIR}/bin/libmp3lame-0.dll" DESTINATION bin COMPONENT dlls)
  install(FILES "${MSYS_DIR}/bin/libopencore-amrnb-0.dll" DESTINATION bin COMPONENT dlls)
  install(FILES "${MSYS_DIR}/bin/libopencore-amrwb-0.dll" DESTINATION bin COMPONENT dlls)
  install(FILES "${MSYS_DIR}/bin/libmodplug-1.dll" DESTINATION bin COMPONENT dlls)
  install(FILES "${MSYS_DIR}/bin/librtmp-1.dll" DESTINATION bin COMPONENT dlls)
  install(FILES "${MSYS_DIR}/bin/zlib1.dll" DESTINATION bin COMPONENT dlls)
  install(FILES "${MSYS_DIR}/bin/libopenjp2-7.dll" DESTINATION bin COMPONENT dlls)
  install(FILES "${MSYS_DIR}/bin/libopus-0.dll" DESTINATION bin COMPONENT dlls)
  install(FILES "${MSYS_DIR}/bin/libspeex-1.dll" DESTINATION bin COMPONENT dlls)
  install(FILES "${MSYS_DIR}/bin/libtheoradec-1.dll" DESTINATION bin COMPONENT dlls)
  install(FILES "${MSYS_DIR}/bin/libtheoraenc-1.dll" DESTINATION bin COMPONENT dlls)
  install(FILES "${MSYS_DIR}/bin/libvorbis-0.dll" DESTINATION bin COMPONENT dlls)
  install(FILES "${MSYS_DIR}/bin/libvorbisenc-2.dll" DESTINATION bin COMPONENT dlls)
  install(FILES "${MSYS_DIR}/bin/libbluray-2.dll" DESTINATION bin COMPONENT dlls)
  install(FILES "${MSYS_DIR}/bin/libpng16-16.dll" DESTINATION bin COMPONENT dlls)
  install(FILES "${MSYS_DIR}/bin/libjpeg-8.dll" DESTINATION bin COMPONENT dlls)
  install(FILES "${MSYS_DIR}/bin/libtiff-5.dll" DESTINATION bin COMPONENT dlls)
  install(FILES "${MSYS_DIR}/bin/libvpx-1.dll" DESTINATION bin COMPONENT dlls)
  install(FILES "${MSYS_DIR}/bin/libwavpack-1.dll" DESTINATION bin COMPONENT dlls)
  install(FILES "${MSYS_DIR}/bin/libwebp-7.dll" DESTINATION bin COMPONENT dlls)
  install(FILES "${MSYS_DIR}/bin/libwebpmux-3.dll" DESTINATION bin COMPONENT dlls)
  install(FILES "${MSYS_DIR}/bin/libwebpdemux-2.dll" DESTINATION bin COMPONENT dlls)
  install(FILES "${MSYS_DIR}/bin/libwebpdecoder-3.dll" DESTINATION bin COMPONENT dlls)
  install(FILES "${MSYS_DIR}/bin/libx265.dll" DESTINATION bin COMPONENT dlls)
  install(FILES "${MSYS_DIR}/bin/xvidcore.dll" DESTINATION bin COMPONENT dlls)
  install(FILES "${MSYS_DIR}/bin/swresample-3.dll" DESTINATION bin COMPONENT dlls)
  install(FILES "${MSYS_DIR}/bin/libx264-157.dll" DESTINATION bin COMPONENT dlls)
  install(FILES "${MSYS_DIR}/bin/libfreetype-6.dll" DESTINATION bin COMPONENT dlls)
  install(FILES "${MSYS_DIR}/bin/libcrypto-1_1-x64.dll" DESTINATION bin COMPONENT dlls)
  install(FILES "${MSYS_DIR}/bin/libfreetype-6.dll" DESTINATION bin COMPONENT dlls)
  install(FILES "${MSYS_DIR}/bin/libgmp-10.dll" DESTINATION bin COMPONENT dlls)
  install(FILES "${MSYS_DIR}/bin/libxml2-2.dll" DESTINATION bin COMPONENT dlls)
  install(FILES "${MSYS_DIR}/bin/libhogweed-4.dll" DESTINATION bin COMPONENT dlls)
  install(FILES "${MSYS_DIR}/bin/libnettle-6.dll" DESTINATION bin COMPONENT dlls)
  install(FILES "${MSYS_DIR}/bin/libogg-0.dll" DESTINATION bin COMPONENT dlls)
  install(FILES "${MSYS_DIR}/bin/libzstd.dll" DESTINATION bin COMPONENT dlls)
  install(FILES "${MSYS_DIR}/bin/libharfbuzz-0.dll" DESTINATION bin COMPONENT dlls)
  install(FILES "${MSYS_DIR}/bin/libglib-2.0-0.dll" DESTINATION bin COMPONENT dlls)
  install(FILES "${MSYS_DIR}/bin/libgraphite2.dll" DESTINATION bin COMPONENT dlls)
  install(FILES "${MSYS_DIR}/bin/libintl-8.dll" DESTINATION bin COMPONENT dlls)
  install(FILES "${MSYS_DIR}/bin/libidn2-0.dll" DESTINATION bin COMPONENT dlls)
  install(FILES "${MSYS_DIR}/bin/libpcre-1.dll" DESTINATION bin COMPONENT dlls)
  install(FILES "${MSYS_DIR}/bin/libp11-kit-0.dll" DESTINATION bin COMPONENT dlls)
  install(FILES "${MSYS_DIR}/bin/libtasn1-6.dll" DESTINATION bin COMPONENT dlls)
  install(FILES "${MSYS_DIR}/bin/libunistring-2.dll" DESTINATION bin COMPONENT dlls)
  install(FILES "${MSYS_DIR}/bin/libffi-6.dll" DESTINATION bin COMPONENT dlls)
  install(FILES "${MSYS_DIR}/bin/libboost_filesystem-mt.dll" DESTINATION bin COMPONENT dlls)
  install(FILES "${PROJECT_BINARY_DIR}/libsrt.dll" DESTINATION bin COMPONENT dlls)

  set(CPACK_PACKAGE_NAME "FPV-NG")
  set(CPACK_PACKAGE_VENDOR "Me")
  set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "FPV")
  set(CPACK_PACKAGE_VERSION "0.1.0")
  set(CPACK_PACKAGE_VERSION_MAJOR "0")
  set(CPACK_PACKAGE_VERSION_MINOR "1")
  set(CPACK_PACKAGE_VERSION_PATCH "0")
  set(CPACK_PACKAGE_INSTALL_DIRECTORY "FPV-NG")

  # Define components and their display names
  set(CPACK_COMPONENTS_ALL applications libraries dlls headers icons fonts configurations)
  set(CPACK_COMPONENT_APPLICATIONS_DISPLAY_NAME "FPV-NG Applications")
  set(CPACK_COMPONENT_LIBRARIES_DISPLAY_NAME "Libraries")
  set(CPACK_COMPONENT_DLLS_DISPLAY_NAME "DLLs")
  set(CPACK_COMPONENT_HEADERS_DISPLAY_NAME "C++ Headers")
  set(CPACK_COMPONENT_ICONS_DISPLAY_NAME "Icons")
  set(CPACK_COMPONENT_FONTS_DISPLAY_NAME "Fonts")

  # Human readable component descriptions
  set(CPACK_COMPONENT_APPLICATIONS_DESCRIPTION
    "An extremely useful application that makes use of FPV-NG")
  set(CPACK_COMPONENT_LIBRARIES_DESCRIPTION
    "Static libraries used to build programs with FPV-NG")
  set(CPACK_COMPONENT_DLLS_DESCRIPTION
    "Shared libraries used to build programs with FPV-NG")
  set(CPACK_COMPONENT_HEADERS_DESCRIPTION
    "C/C++ header files for use with FPV-NG")
  set(CPACK_COMPONENT_ICONS_DESCRIPTION
    "Icon image files for use with FPV-NG")
  set(CPACK_COMPONENT_FONTS_DESCRIPTION
    "Font files for use with FPV-NG")

  # Define dependencies between components
  set(CPACK_COMPONENT_HEADERS_DEPENDS libraries)
  set(CPACK_COMPONENT_APPLICATIONS_DEPENDS dlls icons fonts)

  # Define groups
  set(CPACK_COMPONENT_APPLICATIONS_GROUP "Runtime")
  set(CPACK_COMPONENT_DLLS_GROUP "Runtime")
  set(CPACK_COMPONENT_ICONS_GROUP "Runtime")
  set(CPACK_COMPONENT_FONTS_GROUP "Runtime")
  set(CPACK_COMPONENT_LIBRARIES_GROUP "Development")
  set(CPACK_COMPONENT_HEADERS_GROUP "Development")

  set(CPACK_COMPONENT_GROUP_DEVELOPMENT_DESCRIPTION
    "All of the tools you'll ever need to develop software")

  # Define NSIS installation types
  set(CPACK_ALL_INSTALL_TYPES Full Developer)
  set(CPACK_COMPONENT_LIBRARIES_INSTALL_TYPES Developer Full)
  set(CPACK_COMPONENT_HEADERS_INSTALL_TYPES Developer Full)
  set(CPACK_COMPONENT_APPLICATIONS_INSTALL_TYPES Full)
  set(CPACK_COMPONENT_DLLS_INSTALL_TYPES Full)
  set(CPACK_COMPONENT_ICONS_INSTALL_TYPES Full)
  set(CPACK_COMPONENT_FONTS_INSTALL_TYPES Full)

  # Add the list of executables taht will be linked onto the start menu
  set(CPACK_PACKAGE_EXECUTABLES "video_stream_player;FPV Viewer")

  # Must be after the last CPACK macros
  include(CPack)

endif ()
