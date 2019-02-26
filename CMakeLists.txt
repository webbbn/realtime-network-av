# CMakeLists files in this project can
# refer to the root source directory of the project as ${HELLO_SOURCE_DIR} and
# to the root binary directory of the project as ${HELLO_BINARY_DIR}.
cmake_minimum_required(VERSION 2.8.11)
project(realtime-network-av)

option(USE_OMX "Build programs that use the OMX library on Raspberry Pi" OFF)
option(USE_RTAUDIO "Build programs that use the RTAudio library" OFF)

if (WIN32)
  set(Extra_Libraries "ws2_32")
endif()

if (USE_OMX)

  include_directories(/opt/vc/include /opt/vc/include/interface/vcos/pthreads /opt/vc/include/interface/vmcs_host/linux)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DSTANDALONE -D__STDC_CONSTANT_MACROS -D__STDC_LIMIT_MACROS -DTARGET_POSIX -D_LINUX -DPIC -D_REENTRANT -D_LARGEFILE64_SOURCE -D_FILE_OFFSET_BITS=64 -U_FORTIFY_SOURCE -DHAVE_LIBOPENMAX=2 -DOMX -DOMX_SKIP64BIT -ftree-vectorize -pipe -DUSE_EXTERNAL_OMX -DHAVE_LIBBCM_HOST -DUSE_EXTERNAL_LIBBCM_HOST -DUSE_VCHIQ_ARM")
  link_directories("/opt/vc/lib")
  add_executable(rpi-camera-server rpi-camera-server.c)
  target_link_libraries(rpi-camera-server openmaxil bcm_host vcos pthread)
endif (USE_OMX)

if (USE_RTAUDIO)

  # RTAudio is in a submodule, so we need git to update the submodule
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

  # Ensure the rtaudio directory was checked out successfully
  if(NOT EXISTS "${PROJECT_SOURCE_DIR}/rtaudio/CMakeLists.txt")
    message(FATAL_ERROR "The submodules were not downloaded! GIT_SUBMODULE was turned off or failed. Please update submodules and try again.")
  endif()

  # Add rtaudio to the list of directories to build
  add_subdirectory(rtaudio)

  # Add the rtaudio directory to the paths
  include_directories("${PROJECT_SOURCE_DIR}/rtaudio")
  link_directories("${PROJECT_SOURCE_DIR}/rtaudio")

  # Build the udp_audio server
  add_executable(udp_audio_server udp_audio_player.cc)
  target_link_libraries(udp_audio_server rtaudio ${Extra_Libraries})

endif()

# To find and use ffmpeg avcodec library
find_path(AVCODEC_INCLUDE_DIR libavcodec/avcodec.h)
find_library(AVCODEC_LIBRARY avcodec)
find_library(AVUTIL_LIBRARY avutil)
find_library(SWSCALE_LIBRARY swscale)

# Find the boost libraries
find_package(Boost REQUIRED COMPONENTS program_options)
include_directories(${Boost_INCLUDE_DIR})

find_package(SDL2)

if (SDL2_FOUND)
  include_directories(${SDL2_INCLUDE_DIRS})
  add_executable(video_stream_player video_stream_player.cc)
  target_link_libraries(video_stream_player ${AVCODEC_LIBRARY} ${AVUTIL_LIBRARY} ${SWSCALE_LIBRARY} ${SDL2_LIBRARIES} ${Boost_LIBRARIES} ${Extra_Libraries})
endif()
