
import sys

from cpython cimport array
from cpython.version cimport PY_MAJOR_VERSION
from libc.string cimport memset

cdef extern from 'netinet_in.h':
    struct sockaddr:
        pass
    struct in_addr:
        unsigned long s_addr
    struct sockaddr_in:
        short sin_family
        unsigned short sin_port
        in_addr sin_addr
        char sin_zero[8]
    struct addrinfo:
        int ai_flags
        int ai_family
        int ai_socktype
        int ai_protocol
        int ai_addrlen
        sockaddr *ai_addr
        char *ai_canonname
        addrinfo *ai_next
    struct sockaddr_storage:
        pass

    int getaddrinfo(const char *node, const char *service, const addrinfo *hints, addrinfo **res)

cdef extern from 'arpa_inet.h':
    enum: AF_INET
    enum: SOCK_DGRAM

    unsigned int htons(unsigned short)
    int inet_pton(int af, const char *src, void *dst)

cdef extern from 'srt.h':
    enum: AI_PASSIVE
    enum: SRT_ERROR
    enum: SRTO_SENDER
    enum: SRTO_RCVSYN
    enum: STRR_LIVE
    enum: SRTT_FILE
    enum: SRTO_TRANSTYPE

    struct SRT_MsgCtrl_:
        int flags;   # Left for future
        int msgttl   # TTL for a message, default -1 (delivered always)
        int inorder  # Whether a message is allowed to supersede partially lost one. Unused in stream and live mode.
        int boundary # 0:mid pkt, 1(01b):end of frame, 2(11b):complete frame, 3(10b): start of frame
        unsigned long srctime  # source timestamp (usec), 0: use internal time     
        unsigned int pktseq    # sequence number of the first packet in received message (unused for sending)
        unsigned int msgno     # message number (output value for both sending and receiving)
    ctypedef SRT_MsgCtrl_ SRT_MSGCTRL

    void srt_startup()
    int srt_socket(int af, int type, int protocol)
    int srt_create_socket();
    int srt_close(int sock)
    int srt_setsockflag(int u, int opt, const void* optval, int optlen)
    int srt_bind(int u, const sockaddr* name, int namelen)
    int srt_listen(int u, int backlog)
    int srt_accept(int u, sockaddr* addr, int* addrlen)
    int srt_connect(int u, const sockaddr* name, int namelen)
    int srt_sendmsg2(int u, const char* buf, int len, SRT_MSGCTRL *mctrl)
    int srt_recvmsg(int u, char *buf, int len);
    int srt_recvmsg2(int u, char *buf, int len, SRT_MSGCTRL *mctrl)
    int srt_setsockopt(int u, int level, int optname, const void* optval, int optlen)
    void srt_cleanup()

af_inet = AF_INET
sock_dgram = SOCK_DGRAM

cdef unicode _text(s):
    if type(s) is unicode:
        # Fast path for most common case(s).
        return <unicode>s

    elif PY_MAJOR_VERSION < 3 and isinstance(s, bytes):
        # Only accept byte strings as text input in Python 2.x, not in Py3.
        return (<bytes>s).decode('ascii')

    elif isinstance(s, unicode):
        # We know from the fast path above that 's' can only be a subtype here.
        # An evil cast to <unicode> might still work in some(!) cases,
        # depending on what the further processing does.  To be safe,
        # we can always create a copy instead.
        return unicode(s)

    else:
        raise TypeError("Could not convert to unicode.")

#use this function to initialize the UDT library
def startup():
   srt_startup()

def create_socket():

    # Create the socket
    sock = srt_create_socket();
    if sock == SRT_ERROR:
        return 0

    # Set some socket options
    cdef int tt = SRTT_FILE
    srt_setsockopt(sock, 0, SRTO_TRANSTYPE, &tt, sizeof(tt))

    # Set the options on the socket
    cdef int yes = 1
    srt_setsockflag(sock, SRTO_RCVSYN, &yes, sizeof(yes))

    return sock

def close(sock):
    srt_close(sock)

def cleanup():
    srt_cleanup()

def wait_for_connect(sock, host, port):

    # Get the remote IP address
    cdef sockaddr_in sa
    sa.sin_family = AF_INET;
    sa.sin_port = htons(port);
    st = inet_pton(AF_INET, host.encode("utf-8"), &sa.sin_addr)
    if st != 1:
        print("Error parsing host name")
        return 0

    # Bind to the port
    st = srt_bind(sock, <sockaddr*>&sa, sizeof(sa));
    if st == SRT_ERROR:
        print("Error binging to the port")
        return 0

    # Wait for a connection
    st = srt_listen(sock, 2)
    if st == SRT_ERROR:
        print("Error listening on the socket")
        return 0

    # Accept the connection
    cdef sockaddr_storage sas
    cdef int sas_size = sizeof(sas)
    fd = srt_accept(sock, <sockaddr*>&sas, &sas_size)

    return fd

def connect(sock, host, port):

    # Get the remote IP address
    cdef sockaddr_in sa
    sa.sin_family = AF_INET;
    sa.sin_port = htons(port);
    cdef hbytes
    st = inet_pton(AF_INET, host.encode("utf-8"), &sa.sin_addr)
    if st != 1:
        print("Error parsing host name")
        return -1

    # Connect to the specified host/port
    st = srt_connect(sock, <sockaddr*>&sa, sizeof(sa))
    if st == SRT_ERROR:
        return False
    return True

def sendmsg(sock, buf, ttl=-1, inorder=True):
    cdef char* bufptr = buf
    cdef SRT_MSGCTRL msgctl
    msgctl.flags = 0 # Not used
    msgctl.msgttl = ttl # ms - 3 frames at 60FPS
    msgctl.inorder = inorder
    msgctl.boundary = 0 # Not used
    msgctl.srctime = 0; # Internal time
    msgctl.pktseq = 0; # Not used for sending
    msgctl.msgno = 0; # Not used for sending or receiving
    st = srt_sendmsg2(sock, bufptr, len(buf), &msgctl)
    if st == SRT_ERROR:
        return False
    return True

def recvmsg(conn, buf):
    cdef char* bufptr = buf;
    return srt_recvmsg(conn, bufptr, len(buf))
