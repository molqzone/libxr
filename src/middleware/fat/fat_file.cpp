#include "fat_file.hpp"

#include <cstring>

namespace LibXR
{

FatFile::FatFile(Fat& fat, const char* path) : fat_(fat)
{
  open_err_ = fat_.FindFile(path, entry_);
  if (open_err_ == ErrorCode::OK)
  {
    start_clus_ = (static_cast<uint32_t>(entry_.clus_hi) << 16) | entry_.clus_lo;
    cur_clus_ = start_clus_;
  }
}

ErrorCode FatFile::AdvanceCursor(uint32_t target_clus_idx)
{
  if (cur_clus_idx_ > target_clus_idx)
  {
    // 后退：重置到首簇重新推进（M3a 简单实现，n 次 Seek 前进/后退为 O(n)）。
    cur_clus_ = start_clus_;
    cur_clus_idx_ = 0;
  }
  while (cur_clus_idx_ < target_clus_idx)
  {
    uint32_t next = 0;
    auto rc = fat_.NextCluster(cur_clus_, next);
    if (rc != ErrorCode::OK)
    {
      return rc;
    }
    if (fat_.IsEoc(next))
    {
      // 目录项声明的 size 超过簇链长度：不一致 / Size exceeds chain length.
      return ErrorCode::CHECK_ERR;
    }
    cur_clus_ = next;
    ++cur_clus_idx_;
  }
  return ErrorCode::OK;
}

ErrorCode FatFile::EnsureSector(uint64_t abs_offset)
{
  const uint32_t bytes_sec = fat_.BytesPerSector();
  const uint64_t sector_offset = abs_offset - (abs_offset % bytes_sec);
  if (sector_valid_ && sector_offset_ == sector_offset)
  {
    return ErrorCode::OK;
  }

  const uint32_t clus_idx = static_cast<uint32_t>(sector_offset / fat_.ClusterSize());
  const uint32_t sec_in_clus =
      static_cast<uint32_t>((sector_offset % fat_.ClusterSize()) / bytes_sec);

  auto rc = AdvanceCursor(clus_idx);
  if (rc != ErrorCode::OK)
  {
    return rc;
  }

  const uint64_t lba = fat_.ClusterToSector(cur_clus_) + sec_in_clus;
  rc = fat_.ReadSector(lba, {sector_, sizeof(sector_)});
  if (rc != ErrorCode::OK)
  {
    return rc;
  }

  sector_valid_ = true;
  sector_offset_ = sector_offset;
  return ErrorCode::OK;
}

ErrorCode FatFile::Read(RawData buf, size_t& len)
{
  if (open_err_ != ErrorCode::OK)
  {
    return open_err_;
  }
  if (buf.addr_ == nullptr)
  {
    return ErrorCode::ARG_ERR;
  }

  const uint32_t bytes_sec = fat_.BytesPerSector();
  size_t want = len;
  if (want > entry_.size - offset_)
  {
    want = static_cast<size_t>(entry_.size - offset_);
  }
  if (want > buf.size_)
  {
    want = buf.size_;
  }

  size_t done = 0;
  while (done < want)
  {
    auto rc = EnsureSector(offset_ + done);
    if (rc != ErrorCode::OK)
    {
      len = done;
      return rc;
    }

    const uint32_t in_sector = static_cast<uint32_t>((offset_ + done) % bytes_sec);
    const size_t chunk =
        (want - done < bytes_sec - in_sector) ? (want - done) : (bytes_sec - in_sector);
    std::memcpy(reinterpret_cast<uint8_t*>(buf.addr_) + done, sector_ + in_sector, chunk);
    done += chunk;
  }

  offset_ += done;
  len = done;
  return ErrorCode::OK;
}

ErrorCode FatFile::Seek(uint64_t offset)
{
  if (open_err_ != ErrorCode::OK)
  {
    return open_err_;
  }
  if (offset > entry_.size)
  {
    return ErrorCode::ARG_ERR;
  }
  offset_ = offset;
  return ErrorCode::OK;
}

}  // namespace LibXR
