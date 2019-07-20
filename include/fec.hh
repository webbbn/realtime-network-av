#ifndef FEC_ENCODER_HH
#define FEC_ENCODER_HH

#include <stdint.h>
#include <memory.h>

#include <vector>
#include <iostream>
#include <set>

#include <fec.h>

typedef enum {
  FEC_PARTIAL,
  FEC_COMPLETE,
  FEC_ERROR
} FECStatus;

class FECEncoder {
public:

  FECEncoder(uint8_t num_blocks = 8, uint8_t num_fec_blocks = 4, uint16_t block_size = 1024,
	     bool interlieved = false);

  void encode(const uint8_t *buf, size_t buf_len);

  std::vector<uint8_t*> &blocks() {
    return m_block_ptrs;
  }

  uint8_t num_blocks() const {
    return m_num_blocks;
  }

  uint8_t num_fec_blocks() const {
    return m_num_fec_blocks;
  }

  uint16_t block_size() const {
    return m_block_size;
  }

  bool interlieved() const {
    return m_interlieved;
  }

private:
  uint8_t m_num_blocks;
  uint8_t m_num_fec_blocks;
  uint16_t m_block_size;
  bool m_interlieved;
  uint32_t m_seq_num;
  std::vector<uint8_t> m_buf;
  std::vector<uint8_t*> m_block_ptrs;
};

struct FECDecoderStats {
  FECDecoderStats() : total_blocks(0), total_packets(0), dropped_blocks(0), dropped_packets(0),
		      lost_sync(0), bytes(0) {}
  size_t total_blocks;
  size_t total_packets;
  size_t dropped_blocks;
  size_t dropped_packets;
  size_t lost_sync;
  size_t bytes;
};

class FECDecoder {
public:

  FECDecoder(uint8_t num_blocks = 8, uint8_t num_fec_blocks = 4, uint16_t block_size = 1024,
	     bool interlieved = false);

  uint8_t num_blocks() const {
    return m_num_blocks;
  }

  uint8_t num_fec_blocks() const {
    return m_num_fec_blocks;
  }

  uint16_t block_size() const {
    return m_block_size;
  }

  bool interlieved() const {
    return m_interlieved;
  }

  std::vector<uint8_t*> &blocks() {
    return m_block_ptrs;
  }

  FECStatus add_block(const uint8_t *buf);

  const FECDecoderStats &stats() const {
    return m_stats;
  }

private:
  uint8_t m_num_blocks;
  uint8_t m_num_fec_blocks;
  uint16_t m_block_size;
  bool m_interlieved;
  uint32_t m_packet_num;
  uint32_t m_prev_seq_num;
  uint8_t m_bad_seq_count;
  std::vector<uint8_t> m_buf;
  std::vector<uint8_t*> m_block_ptrs;
  std::vector<uint8_t*> m_fec_ptrs;
  std::set<uint8_t> m_set_blocks;
  FECDecoderStats m_stats;

  void reset(uint32_t pn);

  bool decode();
};

#endif //FEC_ENCODER_HH
