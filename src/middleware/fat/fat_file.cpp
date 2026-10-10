#include "fat_file.hpp"

#include <cstring>

namespace LibXR
{

FatFile::FatFile(Fat& fat, const char* path, Mode mode) : fat_(fat), mode_(mode)
{
  // 记录父目录与键（Sync 回写目录项用）/ Remember parent + key for Sync.
  char parent[256];
  char name[128];
  size_t len = std::strlen(path);
  while (len > 1 && path[len - 1] == '/')
  {
    --len;
  }
  size_t split = len;
  while (split > 0 && path[split - 1] != '/')
  {
    --split;
  }
  const size_t parent_len = (split == 1) ? 1 : split - 1;
  if (split == 0 || parent_len >= sizeof(parent) || len - split >= sizeof(name))
  {
    open_err_ = ErrorCode::ARG_ERR;
    return;
  }
  std::memcpy(parent, path, parent_len);
  parent[parent_len] = '\0';
  std::memcpy(name, path + split, len - split);
  name[len - split] = '\0';

  parent_clus_ = 0;
  parent_root16_ = false;
  open_err_ = fat_.ResolveDir(parent, parent_clus_, parent_root16_);
  if (open_err_ != ErrorCode::OK)
  {
    return;
  }

  FatDirEntry existing;
  const bool found = fat_.FindFile(path, existing) == ErrorCode::OK;

  if (mode == Mode::READ_ONLY)
  {
    if (!found)
    {
      open_err_ = ErrorCode::NOT_FOUND;
      return;
    }
    entry_ = existing;
  }
  else
  {
    // 写模式：已存在按语义截断；不存在则创建。
    // Write mode: truncate existing (per mode) or create.
    open_err_ = fat_.CreateFile(path, entry_);
    if (open_err_ != ErrorCode::OK)
    {
      return;
    }
    if (entry_.attr & FAT_ATTR_DIR)
    {
      open_err_ = ErrorCode::ARG_ERR;  // 目录不能当文件 / Not a file
      return;
    }

    const uint32_t old_start =
        (static_cast<uint32_t>(entry_.clus_hi) << 16) | entry_.clus_lo;
    const uint64_t old_size = entry_.size;

    if (mode == Mode::WRITE_NEW && (old_start >= 2 || old_size != 0))
    {
      if (old_start >= 2)
      {
        open_err_ = fat_.ClusterFreeChain(old_start);
        if (open_err_ != ErrorCode::OK)
        {
          return;
        }
      }
      entry_.clus_hi = 0;
      entry_.clus_lo = 0;
      entry_.size = 0;
      open_err_ = fat_.RewriteDirEntry(parent_clus_, parent_root16_, existing, entry_);
      if (open_err_ != ErrorCode::OK)
      {
        return;
      }
      open_err_ = fat_.SyncFsInfo();
      if (open_err_ != ErrorCode::OK)
      {
        return;
      }
    }
  }

  start_clus_ = (static_cast<uint32_t>(entry_.clus_hi) << 16) | entry_.clus_lo;
  cur_clus_ = start_clus_;
  offset_ = (mode == Mode::WRITE_APPEND) ? entry_.size : 0;
  disk_entry_ = entry_;
}

ErrorCode FatFile::AdvanceCursor(uint32_t target_clus_idx)
{
  if (cur_clus_idx_ > target_clus_idx)
  {
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
      return ErrorCode::CHECK_ERR;
    }
    cur_clus_ = next;
    ++cur_clus_idx_;
  }
  return ErrorCode::OK;
}

ErrorCode FatFile::EnsureClusters(uint32_t clus_idx_end)
{
  if (start_clus_ < 2)
  {
    // 空文件：分配首簇 / Empty file: allocate the first cluster.
    uint32_t first = 0;
    auto rc = fat_.ClusterAlloc(first);
    if (rc != ErrorCode::OK)
    {
      return rc;
    }
    start_clus_ = first;
    cur_clus_ = first;
    cur_clus_idx_ = 0;
    entry_.clus_lo = static_cast<uint16_t>(first & 0xFFFF);
    entry_.clus_hi = static_cast<uint16_t>(first >> 16);

    // 零填充新簇，保证空洞读回为 0 / Zero-fill so sparse gaps read as zero.
    uint8_t zero[512] = {};
    const uint32_t sec_per_clus = fat_.SectorsPerCluster();
    for (uint32_t i = 0; i < sec_per_clus; ++i)
    {
      rc = fat_.WriteSector(fat_.ClusterToSector(first) + i, {zero, sizeof(zero)});
      if (rc != ErrorCode::OK)
      {
        return rc;
      }
    }

    if (clus_idx_end == 0)
    {
      return ErrorCode::OK;
    }
  }

  // 数当前链长，不足则从链尾追加 / Count chain; extend from tail if short.
  bool overflow = false;
  uint32_t count = fat_.CountClusterChain(start_clus_, clus_idx_end + 1, overflow);
  if (overflow)
  {
    return ErrorCode::CHECK_ERR;  // 链成环或断裂 / Looping or broken chain
  }

  if (count > clus_idx_end)
  {
    return ErrorCode::OK;  // 已覆盖 / Already covered
  }

  // 找到链尾（共 count 簇）/ Find the chain tail (count clusters).
  uint32_t tail = start_clus_;
  for (uint32_t i = 1; i < count; ++i)
  {
    uint32_t next = 0;
    auto rc = fat_.NextCluster(tail, next);
    if (rc != ErrorCode::OK)
    {
      return rc;
    }
    tail = next;
  }

  while (count <= clus_idx_end)
  {
    uint32_t fresh = 0;
    auto rc = fat_.ClusterChainAppend(tail, fresh);
    if (rc != ErrorCode::OK)
    {
      return rc;
    }
    uint8_t zero[512] = {};
    const uint32_t sec_per_clus = fat_.SectorsPerCluster();
    for (uint32_t i = 0; i < sec_per_clus; ++i)
    {
      rc = fat_.WriteSector(fat_.ClusterToSector(fresh) + i, {zero, sizeof(zero)});
      if (rc != ErrorCode::OK)
      {
        return rc;
      }
    }
    tail = fresh;
    ++count;
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
  sector_dirty_ = false;
  sector_offset_ = sector_offset;
  return ErrorCode::OK;
}

ErrorCode FatFile::FlushSector()
{
  if (!sector_dirty_ || !sector_valid_)
  {
    return ErrorCode::OK;
  }

  const uint32_t bytes_sec = fat_.BytesPerSector();
  const uint32_t sec_in_clus =
      static_cast<uint32_t>((sector_offset_ % fat_.ClusterSize()) / bytes_sec);
  const uint64_t lba = fat_.ClusterToSector(cur_clus_) + sec_in_clus;

  auto rc = fat_.WriteSector(lba, {sector_, sizeof(sector_)});
  if (rc != ErrorCode::OK)
  {
    return rc;
  }
  sector_dirty_ = false;
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

ErrorCode FatFile::Write(ConstRawData buf, size_t& len)
{
  if (open_err_ != ErrorCode::OK)
  {
    return open_err_;
  }
  if (mode_ == Mode::READ_ONLY)
  {
    return ErrorCode::ARG_ERR;
  }
  if (buf.addr_ == nullptr && len != 0)
  {
    return ErrorCode::ARG_ERR;
  }

  const uint32_t bytes_sec = fat_.BytesPerSector();
  const uint32_t cluster_size = fat_.ClusterSize();
  size_t want = (len < buf.size_) ? len : buf.size_;

  size_t done = 0;
  while (done < want)
  {
    const uint64_t target = offset_ + done;
    const uint32_t clus_idx = static_cast<uint32_t>(target / cluster_size);

    auto rc = EnsureClusters(clus_idx);
    if (rc != ErrorCode::OK)
    {
      len = done;
      return rc;
    }

    rc = EnsureSector(target);
    if (rc != ErrorCode::OK)
    {
      len = done;
      return rc;
    }

    const uint32_t in_sector = static_cast<uint32_t>(target % bytes_sec);
    const size_t chunk =
        (want - done < bytes_sec - in_sector) ? (want - done) : (bytes_sec - in_sector);
    std::memcpy(sector_ + in_sector, reinterpret_cast<const uint8_t*>(buf.addr_) + done,
                chunk);
    sector_dirty_ = true;
    done += chunk;

    // 跨越扇区前先落盘 / Flush before moving to another sector.
    if ((target + chunk) % bytes_sec == 0)
    {
      rc = FlushSector();
      if (rc != ErrorCode::OK)
      {
        len = done;
        return rc;
      }
    }
  }

  offset_ += done;
  if (offset_ > entry_.size)
  {
    entry_.size = offset_;
  }
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

ErrorCode FatFile::Truncate(uint64_t size)
{
  if (open_err_ != ErrorCode::OK)
  {
    return open_err_;
  }
  if (mode_ == Mode::READ_ONLY)
  {
    return ErrorCode::ARG_ERR;
  }

  if (size < entry_.size)
  {
    const uint32_t cluster_size = fat_.ClusterSize();
    const uint32_t keep =
        (size == 0) ? 0 : static_cast<uint32_t>((size + cluster_size - 1) / cluster_size);
    if (start_clus_ >= 2 && keep == 0)
    {
      auto rc = fat_.ClusterFreeChain(start_clus_);
      if (rc != ErrorCode::OK)
      {
        return rc;
      }
      start_clus_ = 0;
    }
    else if (start_clus_ >= 2)
    {
      auto rc = fat_.TruncateChain(start_clus_, keep);
      if (rc != ErrorCode::OK)
      {
        return rc;
      }
    }
    entry_.size = size;
    if (offset_ > size)
    {
      offset_ = size;
    }
  }

  return Sync();
}

ErrorCode FatFile::Sync()
{
  if (open_err_ != ErrorCode::OK)
  {
    return open_err_;
  }

  auto rc = FlushSector();
  if (rc != ErrorCode::OK)
  {
    return rc;
  }

  sector_valid_ = false;

  if (mode_ != Mode::READ_ONLY)
  {
    // 回写 size/起始簇；key 为最近一次落盘状态 / Key = last synced state.
    entry_.clus_lo = static_cast<uint16_t>(start_clus_ & 0xFFFF);
    entry_.clus_hi = static_cast<uint16_t>(start_clus_ >> 16);
    rc = fat_.RewriteDirEntry(parent_clus_, parent_root16_, disk_entry_, entry_);
    if (rc != ErrorCode::OK)
    {
      return rc;
    }
    disk_entry_ = entry_;
    rc = fat_.SyncFsInfo();
    if (rc != ErrorCode::OK)
    {
      return rc;
    }
  }
  return ErrorCode::OK;
}

}  // namespace LibXR
