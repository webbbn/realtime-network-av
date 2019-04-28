# realtime_network_av

A collection of utilities for real-time encoding and decoding audio and video streams.

# Initialize submodules

`git submodule update --init --recursive`

# Install dependent packages

sudo apt-get install cmake libasound2-dev libboost-all-dev libavcodec-dev libavformat-dev libswscale-dev

# Compile SDL on Raspberry Pi that displays on the framebuffer (only works on Raspberry Pi)

*Derived from:* https://choccyhobnob.com/sdl2-2-0-8-on-raspberry-pi/

~~~
sudo apt-get remove libsdl2-dev
sudo apt-get autoremove -y
sudo apt-get install libfontconfig-dev qt5-default automake mercurial libtool libfreeimage-dev libopenal-dev libpango1.0-dev libsndfile-dev libudev-dev libtiff5-dev libwebp-dev libasound2-dev libaudio-dev libxrandr-dev libxcursor-dev libxi-dev libxinerama-dev libxss-dev libesd0-dev freeglut3-dev libmodplug-dev libsmpeg-dev libjpeg-dev libpng16-dev

hg clone http://hg.libsdl.org/SDL

cd SDL
./autogen.sh
./configure --prefix=/home/webbb/sdl_build --disable-pulseaudio --disable-esd --disable-video-mir --disable-video-wayland --disable-video-opengl --host=arm-raspberry-linux-gnueabihf
make -j 3 install
cd ..

wget http://www.libsdl.org/projects/SDL_image/release/SDL2_image-2.0.2.tar.gz
wget http://www.libsdl.org/projects/SDL_mixer/release/SDL2_mixer-2.0.2.tar.gz
wget http://www.libsdl.org/projects/SDL_net/release/SDL2_net-2.0.1.tar.gz
wget http://www.libsdl.org/projects/SDL_ttf/release/SDL2_ttf-2.0.14.tar.gz

tar zxvf SDL2_image-2.0.2.tar.gz
tar zxvf SDL2_mixer-2.0.2.tar.gz
tar zxvf SDL2_net-2.0.1.tar.gz
tar zxvf SDL2_ttf-2.0.14.tar.gz

for D in SDL2_image-2.0.2 SDL2_mixer-2.0.2 SDL2_net-2.0.1 SDL2_ttf-2.0.14; do
  cd ${D}
  ./autogen.sh
  ./configure --prefix=/home/webbb/sdl_build
  make -j 4 install
  cd ..
done
~~~

## Otherwise, install SDL2 libraries

sudo apt-get install libsdl2-dev libsdl2-image-dev libsdl2-ttf-dev

# Compile the code

mkdir build
cd build
cmake -DCMAKE_CXX_FLAGS="-O3 -DNDEBUG -std=c++11" -DCMAKE_INSTALL_PREFIX=/home/webbb/realtime-network-av/install ..

