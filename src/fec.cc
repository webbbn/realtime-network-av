
#include <fec.hh>

FECDecoder::FECDecoder(uint8_t num_blocks, uint8_t num_fec_blocks, uint16_t block_size,
		       bool interlieved) :
  m_num_blocks(num_blocks), m_num_fec_blocks(num_fec_blocks), m_block_size(block_size),
  m_interlieved(interlieved), m_buf((num_blocks + num_fec_blocks) * block_size),
  m_block_ptrs(num_blocks + num_fec_blocks), m_packet_num(0), m_prev_seq_num(0),
  m_bad_seq_count(0) {
  for (size_t i = 0; i < (num_blocks + num_fec_blocks); ++i) {
    m_block_ptrs[i] = m_buf.data() + i * block_size;
  }
}

FECStatus FECDecoder::add_block(const uint8_t *buf) {
  uint32_t nblocks = m_num_blocks + m_num_fec_blocks;

  // The first 32 bits in the data should be a sequence number.
  uint32_t seq_num = ((uint32_t*)buf)[0];

  // Make sure the sequence number is reasonable.
  if ((m_prev_seq_num != 0) && ((seq_num < m_prev_seq_num) || (seq_num > (m_prev_seq_num + 10)))) {
    if (++m_bad_seq_count > 8) {
      reset(0);
      return FEC_ERROR;
    }
    return FEC_PARTIAL;
  }
  m_prev_seq_num = seq_num;

  // The packet number is the sequence number / number of blocks in a packet
  uint32_t pn = seq_num / nblocks;
  // The block number is the modulo
  uint32_t bn = seq_num % nblocks;

  // Is this block associated with the current packet?
  if (pn == m_packet_num) {
    uint32_t ibn = bn;
    // Are the data blocks and FEC blocks interleaved?
    if (m_interlieved) {
      if (bn < (m_num_fec_blocks * 2)) {
	ibn = bn / 2 + (bn % 2) * m_num_blocks;
      } else {
	ibn = (bn - m_num_fec_blocks);
      }
    }
    memcpy(m_block_ptrs[ibn], buf + sizeof(uint32_t), m_block_size);
    m_set_blocks.insert(ibn);
  } else if (pn < m_packet_num) {
    // We likely have decoded early if we already incremented the packet number.
    return FEC_PARTIAL;
  } else {
    // We must not have received enough blocks to decode this packet.
    reset(pn);
    return FEC_ERROR;
  }

  // Decode once we've received anough packets.
  if (m_set_blocks.size() == m_num_blocks) {

    // Decoce the block.
    decode();

    // Start waiting for blocks from the next packet.
    reset(pn + 1);

    // Assume success
    return FEC_COMPLETE;
  }
  return FEC_PARTIAL;
}

bool FECDecoder::decode() {

  // Create the erased blocks array and the FEC blocks array
  uint8_t count = 0;
  std::vector<unsigned int> erased_block_idxs;
  for (size_t i = 0; i < m_num_blocks; ++i) {
    if (m_set_blocks.find(i) == m_set_blocks.end()) {
      erased_block_idxs.push_back(i);
    }
  }
  std::vector<unsigned int> fec_block_idxs;
  std::vector<uint8_t*> fec_blocks;
  for (size_t i = 0; i < m_num_fec_blocks; ++i) {
    if (m_set_blocks.find(i + m_num_blocks) != m_set_blocks.end()) {
      fec_blocks.push_back(m_block_ptrs.data()[i + m_num_blocks]);
      fec_block_idxs.push_back(i);
    }
  }

  // Decode the blocks
  fec_decode(m_block_size,
	     m_block_ptrs.data(),
	     m_num_blocks,
	     fec_blocks.data(),
	     fec_block_idxs.data(),
	     erased_block_idxs.data(),
	     erased_block_idxs.size());
}
