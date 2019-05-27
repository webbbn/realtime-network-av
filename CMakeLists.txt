# CMakeLists files in this project can
# refer to the root source directory of the project as ${HELLO_SOURCE_DIR} and
# to the root binary directory of the project as ${HELLO_BINARY_DIR}.
cmake_minimum_required(VERSION 2.8.11)
project(realtime-network-av)

option(USE_OMX "Build programs that use the OMX library on Raspberry Pi" OFF)

if (WIN32)
  set(Extra_Libraries "ws2_32")
else()
  set(Extra_Libraries "pthread")
endif()

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 -DNDEBUG")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -DNDEBUG")
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

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
  install(TARGETS rpi-camera-server DESTINATION bin)
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
find_package(Boost REQUIRED COMPONENTS program_options system)
include_directories(${Boost_INCLUDE_DIR})

# Build the udp_audio server
add_executable(udp_audio_player src/udp_audio_player.cc)
target_link_libraries(udp_audio_player rtaudio ${Extra_Libraries} ${Boost_LIBRARIES})
install(TARGETS udp_audio_player DESTINATION bin)

add_subdirectory(modules/dcadec)

# To find and use ffmpeg avcodec library
find_path(AVCODEC_INCLUDE_DIR libavcodec/avcodec.h)
find_library(AVCODEC_LIBRARY avcodec)
find_library(AVFORMAT_LIBRARY avformat)
find_library(AVUTIL_LIBRARY avutil)
find_library(SWSCALE_LIBRARY swscale)
include_directories(${AVCODEC_INCLUDE_DIR})
find_path(DCADEC_INCLUDE_DIR libdcadec/common.h PATHS ${PROJECT_SOURCE_DIR}/modules/dcadec)
find_library(DCADEC_LIBRARY dcacec PATHS ${PROJECT_SOURCE_DIR}/modules/dcadec)
include_directories(${DCADEC_INCLUDE_DIR})

find_package(SDL2)
find_library(SDL2_TTF_LIBRARY SDL2_ttf)
find_library(SDL2_IMAGE_LIBRARY SDL2_image)

# Add the mavlink directory to the include path
include_directories(${PROJECT_SOURCE_DIR}/modules/mavlink/common)

include_directories(${SDL2_INCLUDE_DIRS})
add_executable(video_stream_player
  src/video_stream_player.cc
  src/telemetry.cc
  src/texture.cc
  src/ffmpeg_decoder.cc
  src/sdl_osd.cc
  src/sdl_render_window.cc
  src/transmitter.cc)
target_link_libraries(video_stream_player ${AVCODEC_LIBRARY} ${AVFORMAT_LIBRARY} ${AVUTIL_LIBRARY} ${SWSCALE_LIBRARY} ${SDL2_LIBRARIES} ${SDL2_TTF_LIBRARY} ${SDL2_IMAGE_LIBRARY} ${Boost_LIBRARIES} ${Extra_Libraries})
install(TARGETS video_stream_player DESTINATION bin)

find_library(V4L_LIBRARY v4l2)
if (V4L_LIBRARY)
  add_executable(v4l_h264_server
    src/v4l_h264_server.cc)
  target_link_libraries(v4l_h264_server ${V4L_LIBRARY} ${Boost_LIBRARIES} ${Extra_Libraries})
  install(TARGETS v4l_h264_server DESTINATION bin)
endif (V4L_LIBRARY)

find_package(ALSA)
if (ALSA_FOUND)
  include_directories(${ALSA_INCLUDE_DIRS})
  add_executable(spdif_to_udp src/spdif_to_udp.cc)
  link_directories(modules/dcadec)
  target_link_libraries(spdif_to_udp dcadec asound ${Boost_LIBRARIES})
  install(TARGETS spdif_to_udp DESTINATION bin)
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
  install(TARGETS rpi_video_player DESTINATION bin)
endif ()

