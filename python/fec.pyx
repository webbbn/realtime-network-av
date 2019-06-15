
import numpy as np
import math
import struct
import zlib
from libc.string cimport memcpy
from libc.stdlib cimport malloc, free
from libcpp.map cimport map
from libcpp.string cimport string

cdef extern from 'fec.h':
    void fec_init()
    void fec_encode(unsigned int blockSize,       # The size of each data/FEC block
		    unsigned char **data_blocks,  # The K data blocks
		    unsigned int nrDataBlocks,    # The number of data blocks (K)
		    unsigned char **fec_blocks,   # Buffers to store he N FEC blocks in
		    unsigned int nrFecBlocks)     # The number of FEC blocks (N)

    void fec_decode(unsigned int blockSize,       # The size of each data/FEC block
		    unsigned char **data_blocks,  # The K data block buffers (some should contain data)
		    unsigned int nr_data_blocks,  # The number of data blocks (K)
		    unsigned char **fec_blocks,   # The (M <= N) FEC blocks
		    unsigned int *fec_block_nos,  # An array of indexes of the FEC blocks passed in
		    unsigned int *erased_blocks,  # An array of indexes of the data blocks that are not valid
		    unsigned short nr_fec_blocks) # The number (M) of FEC blocks passed in.

cdef class FECCode:
    cdef unsigned char k
    cdef unsigned char n

    def __init__(self, k, n):
        self.k = k
        self.n = n
        fec_init()

    def header_size(self):
        return 7;

    # Encode a message to a set of K blocks and N FEC blocks
    def encode(self, msg):
        msglen = len(msg)
        blocks = []
        block_size = math.ceil(msglen / self.k)

        # Create the blocks out of the message
        for i in range(0, msglen, block_size):
            if (i + block_size) < msglen:
                length = struct.pack("=H", block_size)
                blocks.append(length + msg[i : i + block_size])
            else:
                length = struct.pack("=H", msglen - i)
                blocks.append(length + msg[i : msglen].ljust(block_size))
        nblocks = len(blocks)

        # Copy the input data block pointers into a C array
        data_blocks = []
        cdef unsigned char **c_data_blocks = <unsigned char **>malloc(sizeof(unsigned char*) * nblocks)
        for i in range(0, nblocks):
            data_blocks.append(blocks[i])
            c_data_blocks[i] = blocks[i]

        # Create the fec arrays
        fec_blocks = []
        cdef unsigned char **c_fec_blocks = <unsigned char **>malloc(sizeof(unsigned char*) * self.n)
        for i in range(0, self.n):
            fec_blocks.append(np.zeros(block_size + 2, dtype=np.uint8).tobytes())
            c_fec_blocks[i] = fec_blocks[i]

        # Encode the blocks
        fec_encode(block_size + 2, c_data_blocks, nblocks, c_fec_blocks, self.n);

        # Cleanup
        free(c_data_blocks)
        free(c_fec_blocks)

        # Return the encoded blocks
        return data_blocks + fec_blocks

    # Encode a frame of data and append a header to each block returned
    def encode_frame(self, msg, frame_id = 0, sub_frame_id = 0, num_sub_frames = 0):

        # Encode the blocks
        fec_blocks = self.encode(msg)
        blocklen = len(fec_blocks[0])

        # Add the header to each of the blocks
        blocks = []
        for i, block in enumerate(fec_blocks):
            header = struct.pack("=HBBB", frame_id % 256, sub_frame_id, num_sub_frames, i)
            sub_msg = header + block
            chksum = zlib.crc32(sub_msg)
            checksum = struct.pack("=I", chksum)
            blocks.append(checksum + sub_msg)

        return blocks

    # Parse the header off of the frame block and verify the checksum
    def parse_frame_block(self, block):

        # Verify the checksum
        chksum = struct.unpack("=I", block[:4])[0]
        if zlib.crc32(block[4:]) != chksum:
            return None

        # Pull out the header fields
        ret = list(struct.unpack("=HBBB", block[4:9]))
        ret.append(block[9:])
        return ret

    # Decode a set of blocks
    def decode(self, fec_blocks):
        # We'll get the block size from the fec_block elements
        blocksize = 0

        # Count the number of data blocks and FEC blocks
        cdef size_t block_cnt = 0;
        cdef size_t fec_block_cnt = 0;
        for i, block in enumerate(fec_blocks):
            if block is not None:
                blocksize = len(block)
                if i < self.k:
                    block_cnt += 1
                else:
                    fec_block_cnt += 1

        # We can't recover the packet if there are not at least K blocks + FEC blocks
        if (block_cnt + fec_block_cnt) < self.k:
            return False

        # Allocate the data block array
        nfec_blocks = self.k - block_cnt
        cdef unsigned char **c_data_blocks = <unsigned char **>malloc(sizeof(unsigned char*) * self.k)
        cdef unsigned int *c_erased_block_idxs = <unsigned int *>malloc(sizeof(unsigned int) * nfec_blocks)
        data_blocks = []
        eb = 0
        for i in range(0, self.k):
            if fec_blocks[i] is None:
                data_blocks.append(np.zeros(blocksize, dtype=np.uint8).tobytes())
                c_erased_block_idxs[eb] = i
                eb += 1
            else:
                data_blocks.append(fec_blocks[i])
            c_data_blocks[i] = data_blocks[i]

        # Copy the input fec block pointers into a C array
        cdef unsigned char **c_fec_blocks = <unsigned char **>malloc(sizeof(unsigned char*) * nfec_blocks)
        cdef unsigned int *c_fec_block_idxs = <unsigned int *>malloc(sizeof(unsigned int) * nfec_blocks)
        fb = 0
        for i in range(0, self.n):
            if fb == nfec_blocks:
                break
            if fec_blocks[self.k + i] is not None:
                c_fec_blocks[fb] = fec_blocks[self.k + i]
                c_fec_block_idxs[fb] = i
                fb += 1

        # Decode the blocks
        fec_decode(blocksize, c_data_blocks, self.k, c_fec_blocks, c_fec_block_idxs, c_erased_block_idxs, nfec_blocks)

        # Cleanup
        free(c_data_blocks)
        free(c_fec_blocks)
        free(c_fec_block_idxs)
        free(c_erased_block_idxs)

        # Return the decoded blocks
        return data_blocks

    # Decode a set of frame blocks
    def decode_frame(self, fec_blocks):
        ret = None

        # Separate the data blocks from the FEC blocks
        block_map = {}
        fec_block_map = {}
        for block in fec_blocks:
            frame_id, sub_frame_id, num_sub_frames, block_id, block_data = block
            if ret is None:
                ret = [frame_id, sub_frame_id, num_sub_frames]
            if block_id < self.k:
                block_map[block_id] = block_data
            else:
                fec_block_map[block_id - self.k] = block_data

        # Fill out the blocks array
        fec_blocks = []
        for i in range(self.k):
            if i in block_map:
                fec_blocks.append(block_map[i])
            else:
                fec_blocks.append(None)
        for i in range(self.n):
            if i in fec_block_map:
                fec_blocks.append(fec_block_map[i])
            else:
                fec_blocks.append(None)

        # Decode the blocks
        dec_blocks = self.decode(fec_blocks)

        # Reconstruct the message
        msg = None
        for i, block in enumerate(dec_blocks):
            if i == 0:
                msg = block[2:]
            elif i == self.k - 1:
                length = struct.unpack("=H", block[:2])[0]
                msg = msg + block[2:length + 2]
            else:
                msg = msg + block[2:]

        ret.append(msg)
        return ret
