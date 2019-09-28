# realtime_network_av

A collection of utilities for real-time encoding and decoding audio and video streams.

# Initialize submodules

~~~
git submodule update --init --recursive
~~~

# Install dependent packages

~~~
sudo apt-get install cmake libasound2-dev libboost-all-dev libavcodec-dev libavformat-dev libswscale-dev libssl-dev cython3 libv4l-dev libpcap-dev libsqlite3-dev python3-pip python3-numpy python-future python-lxml python3-future python3-lxml python3-setuptools python3-serial python3-wheel python3-pyudev
sudo -H pip3 install pymavlink pyric
~~~

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
wget https://www.libsdl.org/projects/SDL_image/release/SDL2_image-2.0.5.tar.gz
wget https://www.libsdl.org/projects/SDL_ttf/release/SDL2_ttf-2.0.15.tar.gz
wget http://www.ferzkopp.net/Software/SDL2_gfx/SDL2_gfx-1.0.4.tar.gz

tar xvf SDL2-2.0.9.tar.gz
tar xvf SDL2_image-2.0.5.tar.gz
tar xvf SDL2_ttf-2.0.15.tar.gz
tar xvf SDL2_gfx-1.0.4.tar.gz

cd SDL2-2.0.9
CFLAGS="-O3 -DNDEBUG" ./configure --prefix=/home/webbb/realtime-network-av/build/sdl --disable-pulseaudio --disable-esd --disable-video-mir --disable-video-wayland --disable-video-opengl --disable-video-directfb --host=arm-raspberry-linux-gnueabihf
make -j4 install
cd ..

for D in SDL2_image-2.0.5 SDL2_ttf-2.0.15 SDL2_gfx-1.0.4; do
  cd ${D}
  CFLAGS="-O3 -DNDEBUG" ./configure --disable-mmx --prefix=/home/webbb/realtime-network-av/build/sdl
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

sudo apt-get install libsdl2-dev libsdl2-image-dev libsdl2-ttf-dev libsdl2-gfx-dev

# Compile the code

cmake -DCMAKE_PREFIX_PATH=`realpath \`pwd\``/sdl -DCMAKE_INSTALL_PREFIX=`realpath \`pwd\`/..`/install ..
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
pip install --upgrade setuptools
~~~
Install pi3d and dependencies
~~~
pip install numpy
pip install pillow
pip install pygame
pip install pi3d
~~~

# Configure Raspberry Pi 3B as WiFi Access Point

~~~
sudo apt-get install hostapd dnsmasq
~~~
Add to the end of /etc/dhcpcd.conf:
~~~
denyinterfaces wlan0
~~~
Add to the end of /etc/network/interfaces
~~~
auto lo
iface lo inet loopback

auto eth0
iface eth0 inet dhcp

allow-hotplug wlan0
iface wlan0 inet static
    address 192.168.128.1
    netmask 255.255.255.0
    network 192.168.128.0
    broadcast 192.168.128.255
~~~
Add the following to /etc/hostapd/hostapd.conf
~~~
interface=wlan0
driver=nl80211
ssid=groundpi
wpa_passphrase=somethingsecret
hw_mode=a
channel=40
ieee80211n=1
wmm_enabled=1
ht_capab=[HT40][SHORT-GI-20][DSSS_CCK-40]
macaddr_acl=0
auth_algs=1
ignore_broadcast_ssid=0
wpa=2
wpa_key_mgmt=WPA-PSK
rsn_pairwise=CCMP
~~~
Edit /etc/default/hostapd
~~~
DAEMON_CONF="/etc/hostapd/hostapd.conf"
~~~
Edit /etc/dnsmasq.conf
~~~
interface=wlan0
listen-address=192.168.128.1
bind-interfaces
server=8.8.8.8
domain-needed
bogus-priv
dhcp-range=192.168.128.100,192.168.128.200,24h
~~~
Enable hostapd
~~~
sudo systemctl unmask hostapd
sudo systemctl enable hostapd
~~~
Restart
~~~
sudo reboot
~~~

# Configure Atheros WiFi adapter in monitor mode
~~~
DEV=wlan1
ifconfig ${DEV} down
rmmod ath9k_htc
sleep 2
modprobe ath9k_htc
sleep 2
ifconfig ${DEV} down
iwconfig ${DEV} mode managed
ifconfig ${DEV} up
iw dev ${DEV} set bitrates legacy-2.4 18
ifconfig ${DEV} down
iw ${DEV} set monitor otherbss fcsfail
#iw reg set 80
ifconfig ${DEV} up
#iwconfig ${DEV} freq 2.472G
iw dev ${DEV} set channel 1
#iwconfig ${DEV} txpower 30
~~~

# Install wifi configuration scripts
~~~
cp conf/wifi_config /etc/default
cp python/configure_wifi.py /usr/local/bin
cp services/wifi_config.service /etc/systemd/system
systemctl enable wifi_config

cp conf/wfb_bridge /etc/default
cp install/bin/wfb_bridge /usr/local/bin
sudo cp services/wfb_bridge.service /etc/systemd/system
sudo cp services/wfb_bridge@.service /etc/systemd/system
systemctl enable wfb_bridge@air (or @ground)
