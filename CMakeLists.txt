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

# We have submodule, so we need git to update the submodule
find_package(Git QUIET)

# Update submodules as needed
if(GIT_FOUND AND EXISTS "${PROJECT_SOURCE_DIR}/.git")
  option(GIT_SUBMODULE "Check submodules during build" ON)
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


# Ensure the rtaudio directory was checked out successfully
if(NOT EXISTS "${PROJECT_SOURCE_DIR}/rtaudio/CMakeLists.txt")
  message(FATAL_ERROR "The submodules were not downloaded! GIT_SUBMODULE was turned off or failed. Please update submodules and try again.")
endif()

# Add rtaudio to the list of directories to build
add_subdirectory(rtaudio)

# Add the rtaudio directory to the paths
include_directories("${PROJECT_SOURCE_DIR}/rtaudio")
link_directories("${PROJECT_SOURCE_DIR}/rtaudio")

# Find the boost libraries
find_package(Boost REQUIRED COMPONENTS program_options system)
include_directories(${Boost_INCLUDE_DIR})

# Build the udp_audio server
add_executable(udp_audio_player src/udp_audio_player.cc)
target_link_libraries(udp_audio_player rtaudio ${Extra_Libraries} ${Boost_LIBRARIES})
install(TARGETS udp_audio_player DESTINATION bin)

add_subdirectory(dcadec)

# To find and use ffmpeg avcodec library
find_path(AVCODEC_INCLUDE_DIR libavcodec/avcodec.h)
find_library(AVCODEC_LIBRARY avcodec)
find_library(AVFORMAT_LIBRARY avformat)
find_library(AVUTIL_LIBRARY avutil)
find_library(SWSCALE_LIBRARY swscale)
include_directories(${AVCODEC_INCLUDE_DIR})
find_path(DCADEC_INCLUDE_DIR libdcadec/common.h PATHS ${PROJECT_SOURCE_DIR}/dcadec)
find_library(DCADEC_LIBRARY dcacec PATHS ${PROJECT_SOURCE_DIR}/dcadec)
include_directories(${DCADEC_INCLUDE_DIR})

find_package(SDL2)
find_library(SDL2_TTF_LIBRARY SDL2_ttf)
find_library(SDL2_IMAGE_LIBRARY SDL2_image)

# Add the mavlink directory to the include path
include_directories(${PROJECT_SOURCE_DIR}/mavlink/common)

include_directories(${SDL2_INCLUDE_DIRS})
add_executable(video_stream_player
  src/video_stream_player.cc
  src/telemetry.cc
  src/texture.cc
  src/ffmpeg_decoder.cc
  src/sdl_osd.cc
  src/sdl_render_window.cc)
target_link_libraries(video_stream_player ${AVCODEC_LIBRARY} ${AVFORMAT_LIBRARY} ${AVUTIL_LIBRARY} ${SWSCALE_LIBRARY} ${SDL2_LIBRARIES} ${SDL2_TTF_LIBRARY} ${SDL2_IMAGE_LIBRARY} ${Boost_LIBRARIES} ${Extra_Libraries})
install(TARGETS video_stream_player DESTINATION bin)
add_executable(vsp src/vsp.cc)
target_link_libraries(vsp ${AVCODEC_LIBRARY} ${AVFORMAT_LIBRARY} ${AVUTIL_LIBRARY} ${SWSCALE_LIBRARY} ${SDL2_LIBRARIES} ${SDL2_TTF_LIBRARY} ${SDL2_IMAGE_LIBRARY} ${Boost_LIBRARIES} ${Extra_Libraries})

find_package(ALSA)
if (ALSA_FOUND)
  include_directories(${ALSA_INCLUDE_DIRS})
  add_executable(spdif_to_udp src/spdif_to_udp.cc)
  link_directories(dcadec)
  target_link_libraries(spdif_to_udp dcadec asound ${Boost_LIBRARIES})
  install(TARGETS spdif_to_udp DESTINATION bin)
endif ()
