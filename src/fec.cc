
#include <math.h>

#include <fec.hh>

FECEncoder::FECEncoder(uint8_t num_blocks, uint8_t num_fec_blocks, uint16_t max_block_size) :
  m_num_blocks(num_blocks), m_num_fec_blocks(num_fec_blocks), m_max_block_size(max_block_size),
  m_seq_num(0), m_block_sizes(num_blocks),
  m_buf((max_block_size + 6) * num_blocks * num_fec_blocks) {

  // Fill the block pointers.
  m_data_blocks.resize(num_blocks);
  m_fec_blocks.resize(num_fec_blocks);
  for (uint8_t i = 0; i < num_blocks; ++i) {
    m_data_blocks[i] = m_buf.data() + (m_max_block_size + 6) * i + 4;
  }
  for (uint8_t i = 0; i < num_fec_blocks; ++i) {
    m_fec_blocks[i] = m_buf.data() + (m_max_block_size + 6) * (i + num_fec_blocks) + 4;
  }

  // Ensure that the FEC library is initialized
  fec_init();
}

// Allocate and initialize the next data block.
std::shared_ptr<FECBlock> FECEncoder::get_next_block(uint16_t length) {
  return std::shared_ptr<FECBlock>(new FECBlock(m_seq_num, m_in_blocks.size(), m_num_blocks,
						m_num_fec_blocks, length));
}

// Add an incoming data block to be encoded
void FECEncoder::add_block(std::shared_ptr<FECBlock> block) {
  FECHeader *h = block->header();
  h->block = m_in_blocks.size();
  m_in_blocks.push_back(block);

  // This block can go out immediately.
  m_out_blocks.push(block);

  // Calculate the FEC blocks when we've received enough blocks.
  if (h->block == (m_num_blocks - 1)) {
    encode_blocks();
  }
}

// Retrieve the next data/fec block
std::shared_ptr<FECBlock> FECEncoder::get_block() {
  if (m_out_blocks.empty()) {
    return std::shared_ptr<FECBlock>();
  }
  std::shared_ptr<FECBlock> ret = m_out_blocks.front();
  m_out_blocks.pop();
  return ret;
}

void FECEncoder::encode_blocks() {

  // Create the FEC arrays of pointers to the data blocks.
  std::vector<uint8_t*> data_blocks(m_num_blocks);
  uint16_t block_size = 0;
  for (uint8_t i = 0; i < m_num_blocks; ++i) {
    data_blocks[i] = m_in_blocks[i]->fec_data();
    block_size = std::max(block_size, static_cast<uint16_t>(m_in_blocks[i]->header()->length + 2));
  }

  // Create the output FEC blocks
  std::vector<uint8_t*> fec_blocks(m_num_fec_blocks);
  for (uint8_t i = 0; i < m_num_fec_blocks; ++i) {
    std::shared_ptr<FECBlock> block(new FECBlock(m_seq_num, m_num_blocks + i, m_num_blocks,
						 m_num_fec_blocks, block_size - 2));
    fec_blocks[i] = block->fec_data();
    block->pkt_length(block_size + sizeof(FECHeader) - 2);
    m_out_blocks.push(block);
  }

  // Encode the blocks.
  fec_encode(block_size, data_blocks.data(), m_num_blocks, fec_blocks.data(), m_num_fec_blocks);

  // Prepare for the next set of blocks.
  ++m_seq_num;
  m_in_blocks.clear();
}

void FECEncoder::encode(const uint8_t *buf, size_t buf_len) {

  // Calculate the number of blocks that will be required for this message.
  size_t block_size = m_max_block_size - 4;
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
      memset(fec_blocks[j], 0, m_max_block_size);
    }

    // Encode the blocks.
    fec_encode(m_max_block_size, data_blocks.data(), m_num_blocks, fec_blocks.data(),
	       m_num_fec_blocks);

    // Insert the block data pointers into the output array. Interlieve if requested.
    size_t oi = i * (m_num_blocks + m_num_fec_blocks);
    for (size_t j = 0; j < m_num_blocks; ++j) {

      // Add the data block to the output array
      uint8_t *dp8 = data_blocks[j] - 4;
      uint32_t *dp32 = reinterpret_cast<uint32_t*>(dp8);
      dp32[0] = m_seq_num++;
      m_block_ptrs[oi++] = dp8;
    }

    // Add the FEC blocks.
    for (size_t j = 0; j < m_num_fec_blocks; ++j) {
      uint8_t *dp8 = fec_blocks[j] - 4;
      uint32_t *dp32 = reinterpret_cast<uint32_t*>(dp8);
      dp32[0] = m_seq_num++;
      m_block_ptrs[oi++] = dp8;
    }
  }
}

FECDecoder::FECDecoder(uint8_t num_blocks, uint8_t num_fec_blocks, uint16_t block_size) :
  m_num_blocks(num_blocks), m_num_fec_blocks(num_fec_blocks), m_block_size(block_size),
  m_packet_num(0), m_prev_seq_num(0), m_bad_seq_count(0),
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
  ++m_stats.total_blocks;

  // The sequence number is the first four bytes of the block.
  const uint32_t *p32 = reinterpret_cast<const uint32_t*>(buf);
  uint32_t seq_num = p32[0];

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
    memcpy(m_block_ptrs[ibn], buf + 4, m_block_size);
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
    if ((*p32 + 4) > m_block_size) {
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


FECDecoder2::FECDecoder2() : m_block_size(0) {
  fec_init();
}

void FECDecoder2::add_block(const uint8_t *buf, uint16_t block_length) {
  std::shared_ptr<FECBlock> blk(new FECBlock(buf, block_length));
  ++m_stats.total_blocks;
  const FECHeader &h = *blk->header();
  FECHeader &ph = m_prev_header;
  uint16_t unrolled_prev_seq = static_cast<uint16_t>(ph.seq_num);
  uint16_t unrolled_seq = static_cast<uint16_t>(h.seq_num);
  if (unrolled_prev_seq > unrolled_seq) {
    unrolled_prev_seq += 256;
  }

  // Did we reach the end of a sequence without getting enough blocks?
  if ((m_block_size != 0) && (unrolled_prev_seq != unrolled_seq)) {

    // Calculate how many packets we dropped with this break in the sequence.
    m_stats.dropped_packets += unrolled_seq - unrolled_prev_seq;

    // Calculate how many packets we dropped.
    uint32_t pseq = unrolled_prev_seq * h.n_blocks + ph.block;
    uint32_t seq = unrolled_seq * h.n_blocks + h.block;
    uint32_t diff = seq - pseq;
    m_stats.dropped_blocks += diff;

    // Reset the sequence.
    m_block_size = 0;
  }

  // Are we starting from a reset or a completed block?
  if (m_block_size == 0) {

    // Initialize the previous packet header to the start of this sequence.
    ph = h;
    ph.block = 0;
    m_blocks.clear();
    m_fec_blocks.clear();
  }

  // The current block size is equal to the block size of the largest block.
  m_block_size = std::max(m_block_size, blk->block_size());

  // Is this a data block or FEC block?
  if (blk->is_data_block()) {

    // Add this block to the list of blocks.
    m_blocks.push_back(blk);

    // Release the block if we don't have a gap.
    if ((m_blocks.size() - 1) == h.block) {
      m_out_blocks.push(blk);
      m_stats.bytes += h.length;
    }

    // Have we reached the end of the data blocks without dropping a packet?
    if (m_blocks.size() == h.n_blocks) {
      m_block_size = 0;
    }

  } else {

    // Add this block to the list of FEC blocks.
    m_fec_blocks.push_back(blk);

    // Decode once we've received anough blocks + FEC blocks and have dropped a block.
    if ((m_blocks.size() + m_fec_blocks.size()) == ph.n_blocks) {

      // Decode the sequence
      decode();
      ++m_stats.total_packets;

      // Start waiting for blocks from the next packet.
      m_block_size == 0;
    }
  }
}

void FECDecoder2::decode() {
  const FECHeader &h = *m_blocks[0]->header();
  uint8_t n_blocks = h.n_blocks;
  uint8_t n_fec_blocks = h.n_fec_blocks;

  // Create the vector of data blocks.
  std::vector<uint8_t*> block_ptrs(h.n_blocks, 0);
  std::vector<std::shared_ptr<FECBlock> > blocks(h.n_blocks);
  for (auto block : m_blocks) {
    blocks[block->header()->block] = block;
    block_ptrs[block->header()->block] = block->fec_data();
  }

  // Create the erased blocks array
  std::vector<std::shared_ptr<FECBlock> > fec_blocks;
  std::vector<unsigned int> erased_block_idxs;
  for (size_t i = 0; i < h.n_blocks; ++i) {
    if (!block_ptrs[i]) {
      std::shared_ptr<FECBlock> blk(new FECBlock(h, m_block_size));
      erased_block_idxs.push_back(i);
      fec_blocks.push_back(blk);
      blocks[i] = blk;
      block_ptrs[i] = blk->fec_data();
    }
  }

  // Create the FEC blocks array
  std::vector<uint8_t*> fec_block_ptrs;
  std::vector<unsigned int> fec_block_idxs;
  for (auto block : m_fec_blocks) {
    uint8_t fec_block_idx = block->header()->block - block->header()->n_blocks;
    fec_block_ptrs.push_back(block->fec_data());
    fec_block_idxs.push_back(fec_block_idx);
  }

  // Decode the blocks
  fec_decode(m_block_size,
	     block_ptrs.data(),
	     n_blocks,
	     fec_block_ptrs.data(),
	     fec_block_idxs.data(),
	     erased_block_idxs.data(),
	     erased_block_idxs.size());

  // Send the remainder of blocks that have a reasonable length.
  for (size_t i = erased_block_idxs[0]; i < n_blocks; ++i) {
    uint16_t length = blocks[i]->data_length();
    if (length <= m_block_size) {
      m_out_blocks.push(blocks[i]);
      m_stats.bytes += length;
    } else {
      ++m_stats.dropped_blocks;
    }
  }
}

// Retrieve the next data/fec block
std::shared_ptr<FECBlock> FECDecoder2::get_block() {
  if (m_out_blocks.empty()) {
    return std::shared_ptr<FECBlock>();
  }
  std::shared_ptr<FECBlock> ret = m_out_blocks.front();
  m_out_blocks.pop();
  return ret;
}
