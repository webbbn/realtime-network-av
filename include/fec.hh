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

class FECDecoder {
public:

  FECDecoder(uint8_t num_blocks = 8, uint8_t num_fec_blocks = 4, uint16_t block_size = 1024,
	     bool interlieved = false);

  uint8_t nblocks() const {
    return m_num_blocks;
  }

  uint8_t nfecblocks() const {
    return m_num_fec_blocks;
  }

  std::vector<uint8_t*> &blocks() {
    return m_block_ptrs;
  }

  FECStatus add_block(const uint8_t *buf);

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

  void reset(uint32_t pn) {
    m_packet_num = pn + 1;
    m_prev_seq_num = 0;
    m_set_blocks.clear();
  }

  bool decode();
};

#endif //FEC_ENCODER_HH
