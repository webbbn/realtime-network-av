
PROGRAMS = rpi-camera-server

all: $(PROGRAMS)

rpi-camera-server : rpi-camera-server.c
	gcc -o rpi-camera-server -DSTANDALONE -D__STDC_CONSTANT_MACROS -D__STDC_LIMIT_MACROS -DTARGET_POSIX -D_LINUX -DPIC -D_REENTRANT -D_LARGEFILE64_SOURCE -D_FILE_OFFSET_BITS=64 -U_FORTIFY_SOURCE -DHAVE_LIBOPENMAX=2 -DOMX -DOMX_SKIP64BIT -ftree-vectorize -pipe -DUSE_EXTERNAL_OMX -DHAVE_LIBBCM_HOST -DUSE_EXTERNAL_LIBBCM_HOST -DUSE_VCHIQ_ARM -I/opt/vc/include -I/opt/vc/include/interface/vcos/pthreads -Iopt/vc/include/interface/vmcs_host/linux -fPIC -ftree-vectorize -pipe -Wall -Werror -O2 -g rpi-camera-server.c -L/opt/vc/lib -lopenmaxil -lbcm_host -lvcos -lpthread

clean:
	rm -f $(PROGRAMS)

.PHONY: all clean
