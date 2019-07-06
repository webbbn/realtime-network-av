
#include <math.h>

#include <fec.hh>

FECEncoder::FECEncoder(uint8_t num_blocks, uint8_t num_fec_blocks, uint16_t block_size,
		       bool interlieved) :
  m_num_blocks(num_blocks), m_num_fec_blocks(num_fec_blocks), m_block_size(block_size),
  m_interlieved(interlieved), m_seq_num(0) {

  // Ensure that the FEC library is initialized
  fec_init();
}

void FECEncoder::encode(const uint8_t *buf, size_t buf_len) {

  // Calculate the number of blocks that will be required for this message.
  size_t block_size = m_block_size - 4;
  size_t sub_frame_size = block_size * m_num_blocks;
  size_t n_sub_frames = ceil(double(buf_len) / sub_frame_size);
  size_t num_blocks = n_sub_frames * (m_num_blocks + m_num_fec_blocks);
  size_t nbytes = num_blocks * (block_size + 8);

  // Create the data buffers for storing the data blocks and FEC blocks
  m_block_ptrs.resize(num_blocks);
  m_buf.resize(nbytes);
  for (size_t i = 0; i < num_blocks; ++i) {
    m_block_ptrs[i] = m_buf.data() + i * (block_size + 8);
  }

  // Divide the frames into sub-frames
  for (size_t i = 0; i < n_sub_frames; ++i) {
    std::vector<uint8_t*> data_blocks(m_num_blocks);
    std::vector<uint8_t*> fec_blocks(m_num_fec_blocks);
    for (size_t j = 0; j < m_num_blocks; ++j) {
      size_t block_num = i * m_num_blocks + j;
      size_t sf_idx = std::min(block_num * block_size, buf_len);
      size_t sf_end = std::min(sf_idx + block_size, buf_len);
      size_t sf_len = sf_end - sf_idx;

      // Get a pointer to this block
      uint8_t *ptr = m_block_ptrs[i * (m_num_blocks + m_num_fec_blocks) + j];

      // First copy the length into the buffer.
      uint32_t *lbuf = reinterpret_cast<uint32_t*>(ptr);
      lbuf[1] = static_cast<uint32_t>(sf_len);

      // Copy the packet data if necessary.
      memcpy(ptr + 8, buf + sf_idx, sf_len);

      // Fill with zeros
      //memset(ptr + 8 + sf_idx, 0, block_size - sf_len);

      // Set the fec block ptr.
      data_blocks[j] = ptr + 4;
    }

    // Add the FEC blocks
    for (size_t j = 0; j < m_num_fec_blocks; ++j) {
      uint8_t *ptr = m_block_ptrs[i * (m_num_blocks + m_num_fec_blocks) + m_num_blocks + j];
      fec_blocks[j] = ptr + 4;
      memset(fec_blocks[j], 0, m_block_size);
    }

    // Encode the blocks.
    fec_encode(m_block_size, data_blocks.data(), m_num_blocks, fec_blocks.data(),
	       m_num_fec_blocks);

    // Insert the block data pointers into the output array. Interlieve if requested.
    size_t oi = i * (m_num_blocks + m_num_fec_blocks);
    for (size_t j = 0; j < m_num_blocks; ++j) {

      // Add the data block to the output array
      uint8_t *dp8 = data_blocks[j] - 4;
      uint32_t *dp32 = reinterpret_cast<uint32_t*>(dp8);
      dp32[0] = m_seq_num++;
      m_block_ptrs[oi++] = dp8;

      // Add the FEC block if interlieving was requested
      if (m_interlieved && (j < m_num_fec_blocks)) {
	uint8_t *dp8 = fec_blocks[j] - 4;
	uint32_t *dp32 = reinterpret_cast<uint32_t*>(dp8);
	dp32[0] = m_seq_num++;
	m_block_ptrs[oi++] = dp8;
      }
    }

    // Add the FEC blocks if interlieving was not requested
    if (!m_interlieved) {
      for (size_t j = 0; j < m_num_fec_blocks; ++j) {
	uint8_t *dp8 = fec_blocks[j] - 4;
	uint32_t *dp32 = reinterpret_cast<uint32_t*>(dp8);
	dp32[0] = m_seq_num++;
	m_block_ptrs[oi++] = dp8;
      }
    }
  }
}

FECDecoder::FECDecoder(uint8_t num_blocks, uint8_t num_fec_blocks, uint16_t block_size,
		       bool interlieved) :
  m_num_blocks(num_blocks), m_num_fec_blocks(num_fec_blocks), m_block_size(block_size),
  m_interlieved(interlieved), m_packet_num(0), m_prev_seq_num(0), m_bad_seq_count(0),
  m_buf((num_blocks + num_fec_blocks) * block_size),
  
  m_block_ptrs(num_blocks + num_fec_blocks) {

  // Ensure that the FEC library is initialized
  fec_init();

  // Create the data buffers for storing the data blocks and FEC blocks
  for (size_t i = 0; i < (num_blocks + num_fec_blocks); ++i) {
    m_block_ptrs[i] = m_buf.data() + i * block_size;
  }
}

FECStatus FECDecoder::add_block(const uint8_t *buf) {
  uint32_t nblocks = m_num_blocks + m_num_fec_blocks;

  // The first 32 bits in the data should be a sequence number.
  uint32_t seq_num = ((uint32_t*)buf)[0];
  ++m_stats.total_blocks;

  // Make sure the sequence number is reasonable.
  if ((m_prev_seq_num != 0) && ((seq_num <= m_prev_seq_num) || (seq_num > (m_prev_seq_num + 10)))) {
    if (++m_bad_seq_count > 8) {
      ++m_stats.lost_sync;
      ++m_stats.dropped_packets;
      ++m_stats.total_packets;
      std::cerr << "reset: " << seq_num << " " << m_prev_seq_num << std::endl;
      reset(0);
      m_prev_seq_num = 0;
      return FEC_ERROR;
    }
    return FEC_PARTIAL;
  }
  if ((m_prev_seq_num > 0) && (seq_num > (m_prev_seq_num + 1))) {
    uint32_t diff = seq_num - m_prev_seq_num - 1;
    m_stats.dropped_blocks += diff;
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
  } else if (pn > m_packet_num) {
    // We must not have received enough blocks to decode this packet.
    ++m_stats.dropped_packets;
    ++m_stats.total_packets;
    reset(pn);
    return FEC_ERROR;
  }

  // Decode once we've received anough packets.
  if (m_set_blocks.size() == m_num_blocks) {

    // Decoce the block.
    bool decode_result = decode();
    ++m_stats.total_packets;

    // Start waiting for blocks from the next packet.
    reset(pn + 1);

    // Assume success
    return decode_result ? FEC_COMPLETE : FEC_ERROR;
  }
  return FEC_PARTIAL;
}

bool FECDecoder::decode() {

  // Create the erased blocks array and the FEC blocks array
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

  // Verify that the lengths are all in range.
  bool error = false;
  size_t bytes = 0;
  for (size_t i = 0; i < m_num_blocks; ++i) {
    uint32_t *p32 = reinterpret_cast<uint32_t*>(m_block_ptrs[i]);
    if (*p32 > (m_block_size - 4)) {
      error = true;
      ++m_stats.dropped_blocks;
    } else {
      bytes += *p32;
    }
  }
  if (error) {
    return false;
  }

  m_stats.bytes += bytes;

  return true;
}

void FECDecoder::reset(uint32_t pn) {
  m_packet_num = pn;
  m_set_blocks.clear();
}
