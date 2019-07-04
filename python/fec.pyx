
import numpy as np
import sys
import math
import struct
import zlib
from libc.string cimport memcpy
from libc.stdlib cimport malloc, free
from libc.stdint cimport uint8_t, uint16_t
from libcpp.map cimport map
from libcpp.string cimport string
from libcpp.vector cimport vector

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

cdef extern from 'fec.hh':
    enum: FEC_PARTIAL
    enum: FEC_COMPLETE
    enum: FEC_ERROR
    cdef uint16_t m_block_size

    cdef cppclass FECDecoder:
        FECDecoder(uint8_t num_blocks , uint8_t num_fec_blocks, uint16_t block_size, uint8_t interlieved)
        FECDecoder()
        uint8_t nblocks()
        uint8_t nfecblocks()
        vector[uint8_t*] blocks()
        int add_block(const uint8_t * buf)

    cdef cppclass FECEncoder:
        FECEncoder(uint8_t num_blocks, uint8_t num_fec_blocks, uint16_t block_size, uint8_t interlieved)
        FECEncoder()
        void encode(const uint8_t *buf, size_t buf_len)
        vector[uint8_t*] blocks()

fec_partial = FEC_PARTIAL
fec_complete = FEC_COMPLETE
fec_error = FEC_ERROR

cdef class PyFECDecode:
    cdef FECDecoder m_dec
    cdef uint16_t m_block_size;

    def __cinit__(self, uint8_t num_blocks, uint8_t num_fec_blocks, uint16_t block_size,
                  uint8_t interlieved):
        self.m_dec = FECDecoder(num_blocks, num_fec_blocks, block_size, interlieved)
        self.m_block_size = block_size

    def add_block(self, buf):
        return self.m_dec.add_block(buf)

    def get_blocks(self):
        ret = []
        for b in self.m_dec.blocks():
            ary = np.asarray(<uint8_t[:self.m_block_size]>b)
            ret.append(ary)
        return ret

cdef class PyFECEncoder:
    cdef FECEncoder m_enc
    cdef uint16_t m_block_size;

    def __cinit__(self, uint8_t num_blocks, uint8_t num_fec_blocks, uint16_t block_size,
                  uint8_t interlieved):
        self.m_enc = FECEncoder(num_blocks, num_fec_blocks, block_size, interlieved)
        self.m_block_size = block_size

    def encode(self, msg):
        self.m_enc.encode(msg, len(msg))

    def get_blocks(self):
        ret = []
        for b in self.m_enc.blocks():
            ary = np.asarray(<uint8_t[:self.m_block_size + 4]>b)
            ret.append(ary)
        return ret
