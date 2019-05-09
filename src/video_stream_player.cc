
#include <sys/types.h>
#include <stdio.h>
#include <iostream>
#include <functional>

#include <boost/program_options.hpp>
#include <boost/asio.hpp>

#include "telemetry.hh"
#include "texture.hh"
#include "ffmpeg_decoder.hh"
#include "sdl_render_window.hh"
#include "transmitter.hh"

namespace po=boost::program_options;
namespace ip=boost::asio::ip;

bool check_for_quit() {
  // Check for the SDL quit event.
  SDL_Event event;
  SDL_PollEvent(&event);
  if (event.type == SDL_QUIT) {
    return true;
  }
  return false;
}

int main(int argc, char* argv[]) {
  std::string hostname;
  std::string port;
  uint32_t packet_size;
  bool use_udp;
  bool fullscreen;
  uint16_t screen;
  std::string url;
  std::string font_file;
  std::string home_dir_icon;
  std::string north_arrow_icon;
  po::options_description desc("Allowed options");
  desc.add_options()
    ("help", "produce help message")
    ("hostname,h", po::value<std::string>(&hostname), "name or IP address of the video server")
    ("port,p", po::value<std::string>(&port), "port number of the video server")
    ("packet_size", po::value<uint32_t>(&packet_size)->default_value(32767),
     "the size of the packet buffer (the maximum size of a packet)")
    ("use_udp,U", po::bool_switch(&use_udp), "use the UDP protocol rather than TCP")
    ("fullscreen,f", po::bool_switch(&fullscreen), "make the render window full screen")
    ("screen,s", po::value<uint16_t>(&screen), "the screen to display the video on")
    ("url,u", po::value<std::string>(&url), "read from the specified URL")
    ("font", po::value<std::string>(&font_file), "the path to the OSD font file")
    ("home_dir_icon", po::value<std::string>(&home_dir_icon),
     "the path to the home direction arrow icon file")
    ("north_arrow_icon", po::value<std::string>(&north_arrow_icon),
     "the path to the north direction arror icon file")
    ;

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).run(), vm);
  po::notify(vm);

  if (vm.count("help")) {
    std::cout << "Usage: options_description [options]\n";
    std::cout << desc;
    return 0;
  }
  bool use_url = (url.length() > 0);

  // Create the boost asio IO context for the network interfaces.
#if BOOST_VERSION < 106600
  boost::asio::io_service io_context;
#else
  boost::asio::io_context io_context;
#endif

  // Initialize SDL for video output and joystick.
  if(SDL_Init(SDL_INIT_VIDEO | SDL_INIT_JOYSTICK)) {
    std::cerr << "Could not initialize SDL - " << SDL_GetError() << std::endl;
    SDL_Quit();
    exit(1);
  }

  // Create the interface to read from the transmitter;
  Transmitter tx;

  // Create the address resolver
  ip::tcp::resolver tcp_resolver(io_context);

  // Create the telemetry class
  std::shared_ptr<Telemetry> telem(new Telemetry(io_context, tx));

  // Create the class for rendering everything
  SDLRenderWindow win(telem, font_file, home_dir_icon, north_arrow_icon, screen, fullscreen);

  // Create the draw callback
  auto draw_cb = [&win](uint32_t width, uint32_t height,
			uint8_t *y_plane, uint8_t *u_plane, uint8_t *v_plane) {
		   win.update(width, height, y_plane, u_plane, v_plane);
		 };

  std::shared_ptr<FFMpegDecoder> dec;
  if (use_url) {

    // Create the decoder class
    dec.reset(new FFMpegDecoder(url, draw_cb));

    while (dec->decode_url() && !check_for_quit()) {}

  } else {

    // Create the decoder class
    dec.reset(new FFMpegDecoder(AV_CODEC_ID_H264, draw_cb));

    // Allocate the packet buffer.
    uint8_t *buffer = new uint8_t[packet_size];

    if (use_udp) {
      boost::asio::ip::udp::endpoint listen_endpoint(boost::asio::ip::address_v4::any(),
						     atoi(port.c_str()));
      ip::udp::socket sock(io_context, listen_endpoint);
      sock.set_option(boost::asio::socket_base::broadcast(true));

      bool done = false;
      int32_t sub_packet_num = -1;
      size_t buffer_idx = 0;
      while (!done) {
	boost::asio::ip::udp::endpoint sender_endpoint;
	size_t recv = sock.receive_from(boost::asio::buffer(buffer, packet_size), sender_endpoint);
	if (recv > 0) {
	  done = !dec->decode(buffer, recv);
	}
	done |= check_for_quit();
      }

    } else {

      ip::tcp::socket sock(io_context);
#if BOOST_VERSION < 106600
      boost::asio::connect(sock, tcp_resolver.resolve({hostname, port}));
#else
      boost::asio::connect(sock, tcp_resolver.resolve(hostname, port));
#endif

      bool done = false;
      while (!done) {
	boost::system::error_code ec;
	size_t recv = boost::asio::read(sock, boost::asio::buffer(buffer, packet_size),
					boost::asio::transfer_at_least(1), ec);
	if (ec) {
	  std::cerr << "Read error" << std::endl;
	} else {
	  done = !dec->decode(buffer, recv);
	}

	done |= check_for_quit();
      }
    }

    delete [] buffer;
  }

  return 0;
}
