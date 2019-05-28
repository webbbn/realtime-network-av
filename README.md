# realtime_network_av

A collection of utilities for real-time encoding and decoding audio and video streams.

# Initialize submodules

`git submodule update --init --recursive`

# Install dependent packages

`sudo apt-get install cmake libasound2-dev libboost-all-dev libavcodec-dev libavformat-dev libswscale-dev`

# Install dependent packages on Raspberry Pi

`sudo apt-get install libomxil-bellagio-dev raspberrypi-kernel-headers libraspberrypi-dev`

# Compile SDL on Raspberry Pi that displays on the framebuffer (only works on Raspberry Pi)

*Derived from:* https://choccyhobnob.com/sdl2-2-0-8-on-raspberry-pi/

~~~
mkdir build
cd build
sudo apt-get remove libsdl2-dev
sudo apt-get autoremove -y
sudo apt-get install libfontconfig-dev qt5-default libfreeimage-dev libopenal-dev libpango1.0-dev libsndfile-dev libudev-dev libtiff5-dev libwebp-dev libasound2-dev libaudio-dev libxrandr-dev libxcursor-dev libxi-dev libxinerama-dev libxss-dev libesd0-dev freeglut3-dev libmodplug-dev libsmpeg-dev libjpeg-dev libpng-dev libdirectfb-dev libdirectfb-bin

wget https://www.libsdl.org/release/SDL2-2.0.9.tar.gz
wget https://www.libsdl.org/projects/SDL_image/release/SDL2_image-2.0.4.tar.gz
wget https://www.libsdl.org/projects/SDL_ttf/release/SDL2_ttf-2.0.15.tar.gz

tar xvf SDL2-2.0.9.tar.gz
tar xvf SDL2_image-2.0.4.tar.gz
tar xvf SDL2_ttf-2.0.15.tar.gz

cd SDL2-2.0.9
CFLAGS="-O3 -DNDEBUG" ./configure --prefix=/home/webbb/realtime-network-av/build/sdl --disable-pulseaudio --disable-esd --disable-video-mir --disable-video-wayland --disable-video-opengl --disable-video-directfb --host=arm-raspberry-linux-gnueabihf
make -j4 install
cd ..

for D in SDL2_image-2.0.4 SDL2_ttf-2.0.15; do
  cd ${D}
  CFLAGS="-O3 -DNDEBUG" ./configure --prefix=/home/webbb/realtime-network-av/build/sdl
  make install
  cd ..
done
~~~

## Install SDL2 on Jetson Nano

~~~
sudo apt-get install libfreetype6-dev libv4l-dev

mkdir build
cd build

wget https://www.libsdl.org/release/SDL2-2.0.9.tar.gz
wget http://www.libsdl.org/projects/SDL_image/release/SDL2_image-2.0.4.tar.gz
wget http://www.libsdl.org/projects/SDL_ttf/release/SDL2_ttf-2.0.15.tar.gz

tar xvf SDL2-2.0.9.tar.gz
tar zxvf SDL2_image-2.0.4.tar.gz
tar zxvf SDL2_ttf-2.0.15.tar.gz

cd SDL2-2.0.9
CFLAGS="-O3 -DNDEBUG" ./configure --prefix=/home/webbb/sdl_build
make -j 10 install

for D in SDL2_image-2.0.4 SDL2_ttf-2.0.15; do
  cd ${D}
  CFLAGS="-O3 -DNDEBUG" ./configure --prefix=/home/webbb/sdl_build
  make -j 10 install
  cd ..
done
~~~

## Otherwise, install SDL2 libraries

sudo apt-get install libsdl2-dev libsdl2-image-dev libsdl2-ttf-dev

# Compile the code

cmake -DCMAKE_CXX_FLAGS="-O3 -DNDEBUG -std=c++11" -DCMAKE_PREFIX_PATH=/home/webbb/realtime-network-av/build/sdl/ -DCMAKE_INSTALL_PREFIX=/home/webbb/realtime-network-av/install ..
make -j 4 install

# Build v4l2rtspserver

~~~
git clone git@github.com:webbbn/v4l2rtspserver.git
cd v4l2rtspserver
~~~
Edit v4l2rtspserver.service.in as necessary
~~~
mkdir build
cd build
sudo make install
cmake ../..
sudo systemctl enable v4l2rtspserver
~~~

# Install the telemetry forwarding script

Edit ../python/vc_comm.py as appropriate
~~~
sudo cp ../python/fc_comm.py /usr/local/bin/
sudo cp ../python/fc_comm.service /lib/systemd/system
sudo systemctl enable fc_comm
~~~

# Install on Windows using msys2

~~~
pacman -S mingw64/mingw-w64-x86_64-cmake mingw64/mingw-w64-x86_64-gst-libav mingw64/mingw-w64-x86_64-SDL2_ttf mingw64/mingw-w64-x86_64-SDL2_image mingw64/mingw-w64-x86_64-SDL2 mingw64/mingw-w64-x86_64-boost pacman mingw64/mingw-w64-x86_64-gcc mingw64/mingw-w64-x86_64-ffmpeg make
cmake -G "MSYS Makefiles" -DCMAKE_CXX_FLAGS="-O3 -DNDEBUG -std=c++11" -DCMAKE_PREFIX_PATH=c:/Users/webbb/projects/realtime-network-av/build/sdl/ -DCMAKE_INSTALL_PREFIX=c:/Users/webbb/projects/realtime-network-av/install ..
~~~

# Install pi3d

## Install pi3d on Windows

*Derived from:* http://pi3d.github.io/html/ReadMe.html#windows
*With DLLs from:* https://github.com/paddywwoof/pi3d_windll

Create a new python virtualenv (from an msys2 shell using standard windows python 3.X)
~~~
py -m venv env
.\env\Scripts\activate
py -m pip install --upgrade pip
pip install --upgrade setup tools
~~~
Install pi3d and dependencies
~~~
pip install numpy
pip install pillow
pip install pygame
pip install pi3d
~~~
