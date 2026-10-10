#include "fat.hpp"

#include <cstdio>
#include <cstring>

#include "fat_dir.hpp"
#include "fat_dir_reader.hpp"
#include "libxr_assert.hpp"

namespace LibXR
{

namespace
{

/// ASCII 大小写不敏感比较（FAT 短名匹配语义）/ Case-insensitive ASCII compare.
bool EqualIgnoreCase(const char* a, const char* b)
{
  while (*a != '\0' && *b != '\0')
  {
    char ca = (*a >= 'A' && *a <= 'Z') ? static_cast<char>(*a + 32) : *a;
    char cb = (*b >= 'A' && *b <= 'Z') ? static_cast<char>(*b + 32) : *b;
    if (ca != cb)
    {
      return false;
    }
    ++a;
    ++b;
  }
  return *a == '\0' && *b == '\0';
}

}  // namespace

Fat::Fat(BlockDevice& dev) : dev_(dev)
{
  mount_err_ = static_cast<int>(Mount());
  mounted_ = (mount_err_ == 0);
}

ErrorCode Fat::Mount()
{
  if (dev_.SectorSize() != 512)
  {
    return ErrorCode::NOT_SUPPORT;
  }

  uint8_t buf[512];
  auto rc = dev_.Read(0, 1, {buf, sizeof(buf)});
  if (rc != ErrorCode::OK)
  {
    return rc;
  }

  uint16_t sig = 0;
  std::memcpy(&sig, buf + FAT_BOOT_SIG_OFFSET, sizeof(sig));
  if (sig != FAT_BOOT_SIG)
  {
    return ErrorCode::INIT_ERR;
  }

  FatBpb bpb;
  std::memcpy(&bpb, buf, sizeof(bpb));

  bytes_sec_ = bpb.bytes_sec;
  if (bytes_sec_ != 512)
  {
    return ErrorCode::NOT_SUPPORT;
  }

  if (bpb.sec_clus == 0 || (bpb.sec_clus & (bpb.sec_clus - 1)) != 0 || bpb.sec_clus > 128)
  {
    return ErrorCode::INIT_ERR;
  }
  sec_clus_ = bpb.sec_clus;

  if (bpb.rsvd_sec == 0)
  {
    return ErrorCode::INIT_ERR;
  }
  fat_start_ = bpb.rsvd_sec;

  num_fats_ = bpb.num_fats;
  if (num_fats_ != 1 && num_fats_ != 2)
  {
    return ErrorCode::INIT_ERR;
  }

  fat_sz_ = bpb.fat_sz16 ? bpb.fat_sz16 : bpb.fat_sz32;
  if (fat_sz_ == 0)
  {
    return ErrorCode::INIT_ERR;
  }

  root_ents_ = bpb.root_ents;
  root_secs_ = (root_ents_ * 32 + bytes_sec_ - 1) / bytes_sec_;
  root_start_ = fat_start_ + static_cast<uint64_t>(num_fats_) * fat_sz_;
  data_start_ = root_start_ + root_secs_;

  total_secs_ = bpb.tot_sec16 ? bpb.tot_sec16 : bpb.tot_sec32;
  if (total_secs_ <= data_start_)
  {
    return ErrorCode::INIT_ERR;
  }

  const uint64_t data_secs = total_secs_ - data_start_;
  total_clusters_ = static_cast<uint32_t>(data_secs / sec_clus_);

  // 类型判定以盘上字段为准（fat_sz16/root_ents 由格式化工具成对设置）；
  // 簇数仅用于拒绝 FAT12。真实世界存在低于 65525 簇的 FAT32 卡（如相机
  // 格式化的小容量卡），簇数法会误判，字段法不会。
  if (total_clusters_ <= FAT12_MAX_CLUSTERS)
  {
    return ErrorCode::NOT_SUPPORT;
  }

  if (bpb.fat_sz16 != 0 && bpb.root_ents != 0)
  {
    type_ = TYPE_FAT16;
  }
  else if (bpb.fat_sz16 == 0 && bpb.root_ents == 0 && bpb.fat_sz32 != 0)
  {
    type_ = TYPE_FAT32;
    root_clus_ = bpb.root_clus;
    if (root_clus_ < 2 || root_clus_ >= total_clusters_ + 2)
    {
      return ErrorCode::INIT_ERR;
    }
  }
  else
  {
    return ErrorCode::INIT_ERR;
  }

  // FAT32: 读 FSInfo（空闲簇计数与下一空闲提示）/ Read FSInfo (FAT32).
  // BPB 的 fs_info 字段即 FSInfo 所在绝对扇区号（通常 6/7），不是相对保留区的偏移。
  // The fs_info BPB field is the ABSOLUTE FSInfo sector number, not an offset
  // into the reserved area.
  if (type_ == TYPE_FAT32 && bpb.fs_info != 0)
  {
    fs_info_lba_ = bpb.fs_info;
    uint8_t fs_buf[512];
    auto fs_rc = ReadSector(fs_info_lba_, {fs_buf, sizeof(fs_buf)});
    if (fs_rc == ErrorCode::OK)
    {
      uint32_t lead = 0;
      uint32_t sig = 0;
      std::memcpy(&lead, fs_buf, 4);
      std::memcpy(&sig, fs_buf + 484, 4);
      if (lead == 0x41615252 && sig == 0x61417272)
      {
        fs_info_present_ = true;
        std::memcpy(&free_count_, fs_buf + 488, 4);
        std::memcpy(&next_free_, fs_buf + 492, 4);
        if (next_free_ < 2 || next_free_ >= total_clusters_ + 2)
        {
          next_free_ = 2;
        }
      }
    }
  }

  // FSInfo 缺失/无效时扫描一次获得准确计数（挂载慢路径）/ Scan if FSInfo invalid.
  if (type_ == TYPE_FAT32 && free_count_ == 0xFFFFFFFF)
  {
    uint32_t free = 0;
    for (uint32_t c = 2; c < total_clusters_ + 2; ++c)
    {
      uint32_t v = 0;
      if (ReadFatEntry(c, v) == ErrorCode::OK && v == 0)
      {
        ++free;
      }
    }
    free_count_ = free;
    next_free_ = 2;
  }

  // FAT[0]/FAT[1] 媒体字节校验（两种类型的第一项都含媒体描述符）。
  uint32_t head = 0;
  rc = ReadFatEntry(0, head);
  if (rc != ErrorCode::OK)
  {
    return rc;
  }
  if ((head & 0xFF) != bpb.media && (head & 0xFF) != 0xF8 && (head & 0xFF) != 0xF0)
  {
    return ErrorCode::CHECK_ERR;
  }

  InvalidateFatCache();
  return ErrorCode::OK;
}

ErrorCode Fat::ReadSector(uint64_t lba, RawData buf)
{
  if (buf.size_ < bytes_sec_)
  {
    return ErrorCode::ARG_ERR;
  }
  return dev_.Read(lba, 1, {buf.addr_, bytes_sec_});
}

ErrorCode Fat::ReadFatEntry(uint32_t clus, uint32_t& value)
{
  const uint32_t entry_size = (type_ == TYPE_FAT32) ? 4 : 2;
  const uint64_t offset = static_cast<uint64_t>(clus) * entry_size;
  const uint64_t lba = fat_start_ + offset / bytes_sec_;
  const auto in_sector = static_cast<uint32_t>(offset % bytes_sec_);

  if (!fat_cache_valid_ || fat_cache_lba_ != lba)
  {
    auto rc =
        dev_.Read(lba, 1, {reinterpret_cast<uint8_t*>(fat_cache_data_), bytes_sec_});
    if (rc != ErrorCode::OK)
    {
      return rc;
    }
    fat_cache_valid_ = true;
    fat_cache_lba_ = lba;
  }

  const auto* bytes = reinterpret_cast<const uint8_t*>(fat_cache_data_);
  if (type_ == TYPE_FAT32)
  {
    std::memcpy(&value, bytes + in_sector, 4);
    value &= 0x0FFFFFFF;
  }
  else
  {
    uint16_t v16 = 0;
    std::memcpy(&v16, bytes + in_sector, 2);
    value = v16;
  }
  return ErrorCode::OK;
}

ErrorCode Fat::NextCluster(uint32_t clus, uint32_t& next)
{
  if (clus < 2 || clus >= total_clusters_ + 2)
  {
    return ErrorCode::CHECK_ERR;
  }

  uint32_t value = 0;
  auto rc = ReadFatEntry(clus, value);
  if (rc != ErrorCode::OK)
  {
    return rc;
  }
  if (IsBadCluster(value))
  {
    return ErrorCode::CHECK_ERR;
  }

  next = value;
  return ErrorCode::OK;
}

ErrorCode Fat::OpenRoot(uint32_t& start_clus, bool& is_root16)
{
  if (!mounted_)
  {
    return static_cast<ErrorCode>(mount_err_);
  }

  if (type_ == TYPE_FAT32)
  {
    start_clus = root_clus_;
    is_root16 = false;
  }
  else
  {
    start_clus = 0;
    is_root16 = true;
  }
  return ErrorCode::OK;
}

ErrorCode Fat::ResolveDir(const char* path, uint32_t& start_clus, bool& is_root16)
{
  if (!mounted_)
  {
    return static_cast<ErrorCode>(mount_err_);
  }
  if (path == nullptr || path[0] != '/')
  {
    return ErrorCode::ARG_ERR;
  }

  auto rc = OpenRoot(start_clus, is_root16);
  if (rc != ErrorCode::OK)
  {
    return rc;
  }

  const char* cursor = path;
  char component[128];

  for (;;)
  {
    while (*cursor == '/')
    {
      ++cursor;
    }
    if (*cursor == '\0')
    {
      return ErrorCode::OK;
    }

    size_t len = 0;
    while (cursor[len] != '\0' && cursor[len] != '/' && len + 1 < sizeof(component))
    {
      component[len] = cursor[len];
      ++len;
    }
    component[len] = '\0';
    cursor += len;

    // "." 与 ".." 在绝对路径语义下无意义（不维护 CWD），显式拒绝而非静默。
    if (std::strcmp(component, ".") == 0 || std::strcmp(component, "..") == 0)
    {
      return ErrorCode::ARG_ERR;
    }

    bool found = false;
    FatDirReader reader(*this, start_clus, is_root16);
    char name[256];
    FatDirEntry entry;

    for (;;)
    {
      rc = reader.Next(name, sizeof(name), entry);
      if (rc == ErrorCode::EMPTY)
      {
        break;
      }
      if (rc != ErrorCode::OK)
      {
        return rc;
      }
      if ((entry.attr & FAT_ATTR_DIR) == 0)
      {
        continue;
      }
      if (EqualIgnoreCase(name, component))
      {
        start_clus = (static_cast<uint32_t>(entry.clus_hi) << 16) | entry.clus_lo;
        is_root16 = false;
        found = true;
        break;
      }
    }

    if (!found)
    {
      return ErrorCode::NOT_FOUND;
    }
  }
}

ErrorCode Fat::FindFile(const char* path, FatDirEntry& entry_out)
{
  if (!mounted_)
  {
    return static_cast<ErrorCode>(mount_err_);
  }
  if (path == nullptr || path[0] != '/')
  {
    return ErrorCode::ARG_ERR;
  }

  // 拆分父目录与文件名 / Split parent directory and file name.
  size_t len = std::strlen(path);
  while (len > 1 && path[len - 1] == '/')
  {
    --len;  // 去掉尾部斜杠 / Strip trailing slashes
  }
  if (len == 0)
  {
    return ErrorCode::ARG_ERR;
  }

  size_t split = len;
  while (split > 0 && path[split - 1] != '/')
  {
    --split;
  }
  if (split == 0)
  {
    return ErrorCode::ARG_ERR;
  }

  char parent[256];
  const size_t parent_len = (split == 1) ? 1 : split - 1;  // "/name" 的父目录是 "/"
  if (parent_len >= sizeof(parent))
  {
    return ErrorCode::ARG_ERR;
  }
  std::memcpy(parent, path, parent_len);
  parent[parent_len] = '\0';

  char name[128];
  const size_t name_len = len - split;
  if (name_len >= sizeof(name))
  {
    return ErrorCode::ARG_ERR;
  }
  std::memcpy(name, path + split, name_len);
  name[name_len] = '\0';

  uint32_t clus = 0;
  bool root16 = false;
  auto rc = ResolveDir(parent, clus, root16);
  if (rc != ErrorCode::OK)
  {
    return rc;
  }

  FatDirReader reader(*this, clus, root16);
  char entry_name[256];
  FatDirEntry entry;

  for (;;)
  {
    rc = reader.Next(entry_name, sizeof(entry_name), entry);
    if (rc == ErrorCode::EMPTY)
    {
      return ErrorCode::NOT_FOUND;
    }
    if (rc != ErrorCode::OK)
    {
      return rc;
    }
    if ((entry.attr & FAT_ATTR_DIR) != 0)
    {
      continue;  // 目录不是文件 / Directories are not files
    }
    if (EqualIgnoreCase(entry_name, name))
    {
      entry_out = entry;
      return ErrorCode::OK;
    }
  }
}

ErrorCode Fat::WriteSector(uint64_t lba, ConstRawData buf)
{
  if (buf.size_ < bytes_sec_)
  {
    return ErrorCode::ARG_ERR;
  }
  return dev_.Write(lba, 1, {buf.addr_, bytes_sec_});
}

ErrorCode Fat::FatEntrySet(uint32_t clus, uint32_t value)
{
  const uint32_t entry_size = (type_ == TYPE_FAT32) ? 4 : 2;
  const uint64_t offset = static_cast<uint64_t>(clus) * entry_size;
  const uint64_t sector_in_fat = offset / bytes_sec_;
  const auto in_sector = static_cast<uint32_t>(offset % bytes_sec_);

  // 两份 FAT 都写（镜像一致性；Linux 挂载会检查）。
  // Update both FAT copies for mirror consistency (Linux mount checks this).
  for (uint32_t copy = 0; copy < num_fats_; ++copy)
  {
    const uint64_t lba = fat_start_ + copy * fat_sz_ + sector_in_fat;
    uint8_t sector[512];
    auto rc = dev_.Read(lba, 1, {sector, bytes_sec_});
    if (rc != ErrorCode::OK)
    {
      return rc;
    }
    if (type_ == TYPE_FAT32)
    {
      const uint32_t v = (value & 0x0FFFFFFF);
      std::memcpy(sector + in_sector, &v, 4);
    }
    else
    {
      const auto v = static_cast<uint16_t>(value & 0xFFFF);
      std::memcpy(sector + in_sector, &v, 2);
    }
    rc = WriteSector(lba, {sector, bytes_sec_});
    if (rc != ErrorCode::OK)
    {
      return rc;
    }
  }

  InvalidateFatCache();
  return ErrorCode::OK;
}

ErrorCode Fat::ClusterAlloc(uint32_t& clus, uint32_t hint)
{
  uint32_t start = hint;
  if (start < 2)
  {
    start = (next_free_ >= 2) ? next_free_ : 2;
  }

  const uint32_t first_clus = 2;
  const uint32_t last_clus = total_clusters_ + 1;

  // 从提示处线性扫描，允许环绕一轮 / Linear scan from hint, wrapping once.
  for (uint32_t i = 0; i <= total_clusters_; ++i)
  {
    const uint32_t c = first_clus + ((start - first_clus + i) % total_clusters_);
    if (c > last_clus)
    {
      continue;
    }
    uint32_t value = 0;
    auto rc = ReadFatEntry(c, value);
    if (rc != ErrorCode::OK)
    {
      return rc;
    }
    if (value == 0)
    {
      rc = FatEntrySet(c, IsEoc(0xFFFFFFFF) ? 0xFFFFFFFF : 0xFFFF);
      if (rc != ErrorCode::OK)
      {
        return rc;
      }
      clus = c;
      next_free_ = c + 1;
      if (free_count_ > 0)
      {
        --free_count_;
      }
      return ErrorCode::OK;
    }
  }
  return ErrorCode::FULL;
}

ErrorCode Fat::ClusterFreeChain(uint32_t start)
{
  uint32_t clus = start;
  uint32_t freed = 0;

  while (clus >= 2 && clus < total_clusters_ + 2)
  {
    uint32_t value = 0;
    auto rc = ReadFatEntry(clus, value);
    if (rc != ErrorCode::OK)
    {
      return rc;
    }
    rc = FatEntrySet(clus, 0);
    if (rc != ErrorCode::OK)
    {
      return rc;
    }
    ++freed;
    if (IsEoc(value) || IsBadCluster(value))
    {
      break;
    }
    clus = value;
    if (freed > total_clusters_)
    {
      return ErrorCode::CHECK_ERR;  // 链成环 / Looping chain
    }
  }

  if (free_count_ != 0xFFFFFFFF)
  {
    free_count_ += freed;
  }
  return ErrorCode::OK;
}

ErrorCode Fat::ClusterChainAppend(uint32_t last, uint32_t& new_clus)
{
  auto rc = ClusterAlloc(new_clus);
  if (rc != ErrorCode::OK)
  {
    return rc;
  }
  return FatEntrySet(last, new_clus);
}

uint32_t Fat::CountClusterChain(uint32_t start, uint32_t limit, bool& overflow)
{
  overflow = false;
  uint32_t clus = start;
  uint32_t count = 0;

  while (clus >= 2 && clus < total_clusters_ + 2)
  {
    ++count;
    if (count > limit)
    {
      overflow = true;
      return count;
    }
    uint32_t value = 0;
    if (ReadFatEntry(clus, value) != ErrorCode::OK || IsEoc(value) || IsBadCluster(value))
    {
      return count;
    }
    clus = value;
  }
  return count;
}

ErrorCode Fat::TruncateChain(uint32_t start, uint32_t keep)
{
  uint32_t clus = start;
  for (uint32_t i = 0; i < keep; ++i)
  {
    if (clus < 2)
    {
      return ErrorCode::OK;
    }
    uint32_t value = 0;
    auto rc = ReadFatEntry(clus, value);
    if (rc != ErrorCode::OK)
    {
      return rc;
    }
    if (IsEoc(value))
    {
      return ErrorCode::OK;
    }
    clus = value;
  }
  if (clus >= 2 && clus < total_clusters_ + 2)
  {
    return ClusterFreeChain(clus);
  }
  return ErrorCode::OK;
}

ErrorCode Fat::SyncFsInfo()
{
  if (!fs_info_present_)
  {
    return ErrorCode::OK;
  }

  uint8_t sector[512];
  auto rc = ReadSector(fs_info_lba_, {sector, sizeof(sector)});
  if (rc != ErrorCode::OK)
  {
    return rc;
  }

  uint32_t lead = 0;
  uint32_t struct_sig = 0;
  std::memcpy(&lead, sector + 0, 4);
  std::memcpy(&struct_sig, sector + 484, 4);
  if (lead != 0x41615252 || struct_sig != 0x61417272)
  {
    return ErrorCode::CHECK_ERR;
  }

  if (free_count_ != 0xFFFFFFFF)
  {
    std::memcpy(sector + 488, &free_count_, 4);
  }
  std::memcpy(sector + 492, &next_free_, 4);

  return WriteSector(fs_info_lba_, {sector, sizeof(sector)});
}

namespace
{

/// 目录写入遍历器：顺序槽位 + 读改写 + 簇链扩展。
/// Directory walker for writes: sequential slots, RMW, chain extension.
class DirWriter
{
 public:
  DirWriter(Fat& fat, uint32_t start_clus, bool is_root16)
      : fat_(fat), cur_clus_(start_clus), is_root16_(is_root16)
  {
  }

  /// 加载第一个扇区 / Load the first sector.
  ErrorCode Begin()
  {
    started_ = false;
    entry_idx_ = 0;
    sector_in_clus_ = 0;
    return Advance();
  }

  /// 推进到下一槽位（Begin 后首次调用返回槽位 0）/ Advance to next slot.
  ErrorCode Advance()
  {
    const uint32_t entries_per_sector = fat_.BytesPerSector() / 32;

    if (!started_ || entry_idx_ >= entries_per_sector)
    {
      entry_idx_ = 0;
      if (is_root16_)
      {
        if (sector_in_clus_ >= fat_.RootSectorCount())
        {
          return ErrorCode::EMPTY;  // 根目录无法扩展（FAT16 定长）/ Flat root full
        }
        sector_lba_ = fat_.RootStartSector() + sector_in_clus_;
        sector_in_clus_++;
      }
      else
      {
        if (started_ && sector_in_clus_ >= fat_.SectorsPerCluster())
        {
          uint32_t next = 0;
          auto rc = fat_.NextCluster(cur_clus_, next);
          if (rc != ErrorCode::OK)
          {
            return rc;
          }
          if (fat_.IsEoc(next))
          {
            return ErrorCode::EMPTY;
          }
          cur_clus_ = next;
          sector_in_clus_ = 0;
        }
        sector_lba_ = fat_.ClusterToSector(cur_clus_) + sector_in_clus_;
        sector_in_clus_++;
      }
      auto rc = fat_.ReadSector(sector_lba_, {sector_, sizeof(sector_)});
      if (rc != ErrorCode::OK)
      {
        return rc;
      }
      started_ = true;
    }
    return ErrorCode::OK;
  }

  /// 读取当前槽位条目 / Read the entry at the current slot.
  void Current(FatDirEntry& entry) const
  {
    std::memcpy(&entry, sector_ + entry_idx_ * 32, 32);
  }

  /// 当前槽位坐标 / Current slot coordinates.
  void Slot(Fat::DirSlot& slot) const
  {
    slot.sector_lba = sector_lba_;
    slot.entry_idx = entry_idx_;
  }

  /// 当前扇区内从当前位置起连续的可用槽位数 / Free slot run in this sector.
  [[nodiscard]] uint32_t FreeRunInSector() const
  {
    const uint32_t entries_per_sector = fat_.BytesPerSector() / 32;
    uint32_t run = 0;
    for (uint32_t i = entry_idx_; i < entries_per_sector; ++i)
    {
      FatDirEntry e;
      std::memcpy(&e, sector_ + i * 32, 32);
      if (e.name[0] != FAT_DIR_FREE && e.name[0] != FAT_DIR_END)
      {
        break;
      }
      ++run;
    }
    return run;
  }

  /// 前进一槽 / Move to the next slot index.
  void NextSlot() { ++entry_idx_; }

  /// 回写一个 32 字节条目到指定槽位 / Write a 32-byte entry at a slot.
  ErrorCode WriteAt(const Fat::DirSlot& slot, const void* entry32)
  {
    uint8_t buf[512];
    auto rc = fat_.ReadSector(slot.sector_lba, {buf, sizeof(buf)});
    if (rc != ErrorCode::OK)
    {
      return rc;
    }
    std::memcpy(buf + slot.entry_idx * 32, entry32, 32);
    return fat_.WriteSector(slot.sector_lba, {buf, sizeof(buf)});
  }

  /// 为目录扩展一个新簇（零初始化），并把位置设到其槽位 0。
  /// Extend the directory with a fresh zeroed cluster.
  ErrorCode Extend(Fat::DirSlot& slot)
  {
    uint32_t new_clus = 0;
    auto rc = fat_.ClusterAlloc(new_clus);
    if (rc != ErrorCode::OK)
    {
      return rc;
    }
    rc = fat_.FatEntrySet(cur_clus_, new_clus);
    if (rc != ErrorCode::OK)
    {
      return rc;
    }

    uint8_t zero[512] = {};
    const uint32_t sec_per_clus = fat_.SectorsPerCluster();
    for (uint32_t i = 0; i < sec_per_clus; ++i)
    {
      rc = fat_.WriteSector(fat_.ClusterToSector(new_clus) + i, {zero, sizeof(zero)});
      if (rc != ErrorCode::OK)
      {
        return rc;
      }
    }

    cur_clus_ = new_clus;
    sector_in_clus_ = 0;
    sector_lba_ = fat_.ClusterToSector(new_clus);
    entry_idx_ = 0;
    started_ = true;
    std::memcpy(sector_, zero, sizeof(sector_));

    slot.sector_lba = sector_lba_;
    slot.entry_idx = 0;
    return ErrorCode::OK;
  }

 private:
  Fat& fat_;
  uint32_t cur_clus_ = 0;
  bool is_root16_ = false;
  uint64_t sector_lba_ = 0;
  uint32_t sector_in_clus_ = 0;
  uint32_t entry_idx_ = 0;
  bool started_ = false;
  uint8_t sector_[512] = {};
};

/// ASCII→UCS-2LE 填充（v1 仅 ASCII；>0x7F 由调用方拒绝）。
/// Fill UCS-2LE from ASCII (caller rejects non-ASCII in M3b v1).
void AsciiToUcs2(const char* src, uint8_t* dst, size_t chars)
{
  for (size_t i = 0; i < chars; ++i)
  {
    const uint16_t c = (src[i] == '\0') ? 0x0000 : static_cast<uint8_t>(src[i]);
    std::memcpy(dst + i * 2, &c, 2);
  }
}

/// 短名校验和（LFN 算法）/ Short-name checksum (LFN algorithm).
uint8_t ShortChecksum(const uint8_t name[11])
{
  uint8_t sum = 0;
  for (size_t i = 0; i < 11; ++i)
  {
    sum = static_cast<uint8_t>(((sum & 1) << 7) + (sum >> 1) + name[i]);
  }
  return sum;
}

/// 生成 8.3 短名（LFN 基础名 + ~n 冲突规避）/ Generate an 8.3 short name.
bool GenerateShortName(Fat& fat, uint32_t dir_clus, bool root16, const char* lfn,
                       uint8_t out[11])
{
  char base[8] = {};
  char ext[4] = {};

  const char* dot = nullptr;
  for (const char* p = lfn; *p != '\0'; ++p)
  {
    if (*p == '.')
    {
      dot = p;
      break;
    }
  }

  size_t base_len = 0;
  for (const char* p = lfn; *p != '\0' && p != dot && base_len < 8; ++p)
  {
    char c = *p;
    if (c >= 'a' && c <= 'z')
    {
      c = static_cast<char>(c - 32);
    }
    if (!((c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '_' || c == '-' ||
          c == '~' || c == '!'))
    {
      continue;  // 丢弃非法字符 / Drop invalid characters
    }
    base[base_len++] = c;
  }

  size_t ext_len = 0;
  if (dot != nullptr)
  {
    for (const char* p = dot + 1; *p != '\0' && ext_len < 3; ++p)
    {
      char c = *p;
      if (c >= 'a' && c <= 'z')
      {
        c = static_cast<char>(c - 32);
      }
      if (!((c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '_' || c == '-' ||
            c == '~' || c == '!'))
      {
        continue;
      }
      ext[ext_len++] = c;
    }
  }

  auto store = [&](const char* b, const char* e)
  {
    for (size_t i = 0; i < 11; ++i)
    {
      out[i] = ' ';
    }
    for (size_t i = 0; i < 8 && b[i] != '\0'; ++i)
    {
      out[i] = static_cast<uint8_t>(b[i]);
    }
    for (size_t i = 0; i < 3 && e[i] != '\0'; ++i)
    {
      out[8 + i] = static_cast<uint8_t>(e[i]);
    }
  };

  // 基础名 ≤8 且不带 ~ n：先试原名 / Try the plain name first.
  bool plain = std::strlen(base) <= 8 && std::strchr(base, '~') == nullptr;
  if (plain)
  {
    store(base, ext);
    uint8_t probe[11];
    std::memcpy(probe, out, 11);
    // 冲突检测：同目录已有相同短名则加 ~n / Conflict: append ~n.
    DirWriter w(fat, dir_clus, root16);
    if (w.Begin() == ErrorCode::OK)
    {
      FatDirEntry entry;
      for (;;)
      {
        if (w.Advance() != ErrorCode::OK)
        {
          break;
        }
        w.Current(entry);
        const bool lfn = (entry.attr & FAT_ATTR_LFN) == FAT_ATTR_LFN;
        if (entry.name[0] == FAT_DIR_FREE || entry.name[0] == FAT_DIR_END)
        {
          w.NextSlot();
          continue;
        }
        if (!lfn && std::memcmp(entry.name, probe, 11) == 0)
        {
          plain = false;
          break;
        }
        w.NextSlot();
      }
    }
    if (plain)
    {
      return true;
    }
  }

  // 截断基础名为 6 字符 + ~n / Truncate base to 6 chars + ~n.
  for (uint32_t n = 1; n <= 999999; ++n)
  {
    char suffix[8];
    std::snprintf(suffix, sizeof(suffix), "~%u", static_cast<unsigned>(n));
    char truncated[9];
    std::snprintf(truncated, sizeof(truncated), "%.6s", base);
    char full[16];
    std::snprintf(full, sizeof(full), "%s%s", truncated, suffix);
    store(full, ext);

    uint8_t probe[11];
    std::memcpy(probe, out, 11);
    DirWriter w(fat, dir_clus, root16);
    if (w.Begin() != ErrorCode::OK)
    {
      return false;
    }
    bool conflict = false;
    FatDirEntry entry;
    for (;;)
    {
      if (w.Advance() != ErrorCode::OK)
      {
        break;
      }
      w.Current(entry);
      if (entry.name[0] != FAT_DIR_FREE && entry.name[0] != FAT_DIR_END &&
          (entry.attr & FAT_ATTR_LFN) != FAT_ATTR_LFN &&
          std::memcmp(entry.name, probe, 11) == 0)
      {
        conflict = true;
        break;
      }
      w.NextSlot();
    }
    if (!conflict)
    {
      return true;
    }
  }
  return false;
}

/// 打包 LFN 条目（逆序物理排列，首条打 0x40）/ Pack LFN entries (reverse order).
size_t PackLfn(const char* name, uint8_t short_checksum, FatLfnEntry* out, size_t max)
{
  const size_t len = std::strlen(name);
  const size_t groups = (len + 12) / 13;
  if (groups == 0 || groups > max)
  {
    return 0;
  }

  for (size_t g = 0; g < groups; ++g)
  {
    FatLfnEntry& e = out[groups - 1 - g];  // 物理顺序：末组在前 / Highest group first
    std::memset(&e, 0, sizeof(e));
    e.seq = static_cast<uint8_t>(g + 1);
    if (g == groups - 1)
    {
      e.seq |= 0x40;
    }
    e.attr = FAT_ATTR_LFN;
    e.type = 0;
    e.checksum = short_checksum;

    // 按 name1(5)/name2(6)/name3(2) 切分填充 / Split into 5/6/2.
    const size_t base = g * 13;
    uint8_t ucs2[26] = {};
    for (size_t i = 0; i < 13; ++i)
    {
      const size_t idx = base + i;
      const uint16_t c = (idx < len) ? static_cast<uint8_t>(name[idx]) : 0;
      std::memcpy(ucs2 + i * 2, &c, 2);
    }
    std::memcpy(e.name1, ucs2, 10);
    std::memcpy(e.name2, ucs2 + 10, 12);
    std::memcpy(e.name3, ucs2 + 22, 4);
  }
  return groups;
}

}  // namespace

ErrorCode Fat::CreateFile(const char* path, FatDirEntry& entry_out)
{
  if (!mounted_)
  {
    return static_cast<ErrorCode>(mount_err_);
  }
  if (path == nullptr || path[0] != '/')
  {
    return ErrorCode::ARG_ERR;
  }

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
  if (split == 0)
  {
    return ErrorCode::ARG_ERR;
  }

  char parent[256];
  const size_t parent_len = (split == 1) ? 1 : split - 1;
  if (parent_len >= sizeof(parent))
  {
    return ErrorCode::ARG_ERR;
  }
  std::memcpy(parent, path, parent_len);
  parent[parent_len] = '\0';

  char name[128];
  const size_t name_len = len - split;
  if (name_len == 0 || name_len >= sizeof(name))
  {
    return ErrorCode::ARG_ERR;
  }
  std::memcpy(name, path + split, name_len);
  name[name_len] = '\0';

  // v1 仅 ASCII 文件名（LFN UCS-2 化的边界）/ ASCII-only names in v1.
  for (const char* p = name; *p != '\0'; ++p)
  {
    if (static_cast<uint8_t>(*p) > 0x7F)
    {
      return ErrorCode::NOT_SUPPORT;
    }
  }

  uint32_t dir_clus = 0;
  bool root16 = false;
  auto rc = ResolveDir(parent, dir_clus, root16);
  if (rc != ErrorCode::OK)
  {
    return rc;
  }

  // 已存在：直接返回现有项（由调用方按模式处理截断/追加）。
  // Exists: return the current entry; the caller applies mode semantics.
  {
    FatDirReader reader(*this, dir_clus, root16);
    char entry_name[256];
    FatDirEntry entry;
    for (;;)
    {
      rc = reader.Next(entry_name, sizeof(entry_name), entry);
      if (rc == ErrorCode::EMPTY)
      {
        break;
      }
      if (rc != ErrorCode::OK)
      {
        return rc;
      }
      if (EqualIgnoreCase(entry_name, name))
      {
        entry_out = entry;
        return ErrorCode::OK;
      }
    }
  }

  uint8_t short_name[11];
  if (!GenerateShortName(*this, dir_clus, root16, name, short_name))
  {
    return ErrorCode::FULL;  // 短名冲突号耗尽 / Short-name tail exhausted
  }

  const uint8_t checksum = ShortChecksum(short_name);

  // 名是否已是合法 8.3（无需 LFN）/ Whether an LFN is needed.
  bool need_lfn = false;
  if (name_len > 12)
  {
    need_lfn = true;
  }
  else
  {
    char upper[128];
    for (size_t i = 0; i <= name_len; ++i)
    {
      upper[i] =
          static_cast<char>(name[i] >= 'a' && name[i] <= 'z' ? name[i] - 32 : name[i]);
    }
    if (std::strcmp(upper, name) != 0)
    {
      need_lfn = true;  // 大小写需 LFN 保留 / Preserve case via LFN
    }
  }

  FatLfnEntry lfn[20] = {};
  size_t lfn_count = 0;
  if (need_lfn)
  {
    lfn_count = PackLfn(name, checksum, lfn, 20);
    if (lfn_count == 0)
    {
      return ErrorCode::ARG_ERR;
    }
  }

  // 分配连续槽位（同扇区内，LFN 组不跨扇区）。
  // Allocate contiguous slots within one sector (LFN group not split).
  DirWriter writer(*this, dir_clus, root16);
  rc = writer.Begin();
  if (rc != ErrorCode::OK)
  {
    // 目录为空链（新目录簇）：Begin 失败只在 FAT16 根满时可能。
    if (rc != ErrorCode::EMPTY)
    {
      return rc;
    }
  }

  const uint32_t slots_needed = static_cast<uint32_t>(lfn_count) + 1;
  Fat::DirSlot slot{};
  bool placed = false;
  FatDirEntry scan{};

  while (!placed)
  {
    rc = writer.Advance();
    if (rc == ErrorCode::EMPTY)
    {
      // 目录链尾：扩展新簇 / Extend the directory chain.
      rc = writer.Extend(slot);
      if (rc != ErrorCode::OK)
      {
        return rc;
      }
      placed = true;
      break;
    }
    if (rc != ErrorCode::OK)
    {
      return rc;
    }

    writer.Current(scan);

    if (scan.name[0] == FAT_DIR_END || scan.name[0] == FAT_DIR_FREE)
    {
      const uint32_t free_run = writer.FreeRunInSector();
      if (free_run >= slots_needed)
      {
        writer.Slot(slot);
        placed = true;
      }
      else
      {
        // 空间不足：跳过本段；0x00 改写为 0xE5 使读者继续向后。
        // Skip the run; convert 0x00 to 0xE5 so readers continue past it.
        if (scan.name[0] == FAT_DIR_END)
        {
          Fat::DirSlot end_slot;
          writer.Slot(end_slot);
          uint8_t patched[32];
          std::memcpy(patched, &scan, 32);
          patched[0] = FAT_DIR_FREE;
          writer.WriteAt(end_slot, patched);
        }
        for (uint32_t i = 0; i < free_run; ++i)
        {
          writer.NextSlot();
        }
      }
    }
    else
    {
      writer.NextSlot();
    }
  }

  // 写入 LFN（数组即物理顺序：最高 seq 在前，seq=1 紧贴短名项）。
  // Write LFN in array order (highest seq first, seq 1 adjacent to the short
  // entry) per the FAT spec; mtools writes the same layout.
  for (size_t i = 0; i < lfn_count; ++i)
  {
    rc = writer.WriteAt(slot, &lfn[i]);
    if (rc != ErrorCode::OK)
    {
      return rc;
    }
    ++slot.entry_idx;
  }

  FatDirEntry short_entry = {};
  std::memcpy(short_entry.name, short_name, 11);
  short_entry.attr = FAT_ATTR_ARCHIVE;
  short_entry.clus_hi = 0;
  short_entry.clus_lo = 0;
  short_entry.size = 0;
  rc = writer.WriteAt(slot, &short_entry);
  if (rc != ErrorCode::OK)
  {
    return rc;
  }

  entry_out = short_entry;
  return ErrorCode::OK;
}

ErrorCode Fat::Delete(const char* path)
{
  if (!mounted_)
  {
    return static_cast<ErrorCode>(mount_err_);
  }

  size_t len = std::strlen(path);
  while (len > 1 && path[len - 1] == '/')
  {
    --len;
  }
  if (len == 1)
  {
    return ErrorCode::ARG_ERR;  // 不能删根 / Cannot delete root
  }

  size_t split = len;
  while (split > 0 && path[split - 1] != '/')
  {
    --split;
  }
  char parent[256];
  const size_t parent_len = (split == 1) ? 1 : split - 1;
  if (parent_len >= sizeof(parent))
  {
    return ErrorCode::ARG_ERR;
  }
  std::memcpy(parent, path, parent_len);
  parent[parent_len] = '\0';

  char name[128];
  const size_t name_len = len - split;
  if (name_len == 0 || name_len >= sizeof(name))
  {
    return ErrorCode::ARG_ERR;
  }
  std::memcpy(name, path + split, name_len);
  name[name_len] = '\0';

  uint32_t dir_clus = 0;
  bool root16 = false;
  auto rc = ResolveDir(parent, dir_clus, root16);
  if (rc != ErrorCode::OK)
  {
    return rc;
  }

  FatDirEntry entry;
  rc = FindFile(path, entry);
  if (rc != ErrorCode::OK)
  {
    return rc;
  }

  const bool is_dir = (entry.attr & FAT_ATTR_DIR) != 0;
  if (is_dir)
  {
    FatDir dir(*this, path);
    char child[256];
    bool child_is_dir = false;
    uint64_t child_size = 0;
    if (dir.Next(child, sizeof(child), child_is_dir, child_size) != ErrorCode::EMPTY)
    {
      return ErrorCode::CHECK_ERR;  // 非空目录 / Not empty
    }
  }

  // 定位短名项及其 LFN 前驱，全部标 0xE5。
  // Locate the short entry and its LFN predecessors; mark all 0xE5.
  const uint8_t want_checksum = ShortChecksum(entry.name);
  DirWriter writer(*this, dir_clus, root16);
  rc = writer.Begin();
  if (rc != ErrorCode::OK)
  {
    return rc;
  }

  bool found = false;
  FatDirEntry scan{};
  FatLfnEntry pending_lfn[20] = {};
  Fat::DirSlot pending_slot[20] = {};
  size_t pending = 0;

  for (;;)
  {
    rc = writer.Advance();
    if (rc == ErrorCode::EMPTY)
    {
      break;
    }
    if (rc != ErrorCode::OK)
    {
      return rc;
    }

    writer.Current(scan);
    Fat::DirSlot slot;
    writer.Slot(slot);

    if ((scan.attr & FAT_ATTR_LFN) == FAT_ATTR_LFN)
    {
      if (pending < 20)
      {
        std::memcpy(&pending_lfn[pending], &scan, 32);
        pending_slot[pending] = slot;
        ++pending;
      }
      writer.NextSlot();
      continue;
    }

    if (scan.name[0] == FAT_DIR_FREE || scan.name[0] == FAT_DIR_END)
    {
      pending = 0;  // 间隙打断 LFN 组 / A gap breaks the LFN group
      writer.NextSlot();
      continue;
    }

    if (std::memcmp(scan.name, entry.name, 11) == 0 && scan.clus_lo == entry.clus_lo &&
        scan.clus_hi == entry.clus_hi && scan.size == entry.size)
    {
      // 校验 LFN 属于本项（checksum 匹配才计入删除）/ Verify LFN ownership.
      uint8_t del[32];
      std::memcpy(del, &scan, 32);
      del[0] = FAT_DIR_FREE;
      rc = writer.WriteAt(slot, del);
      if (rc != ErrorCode::OK)
      {
        return rc;
      }
      for (size_t i = 0; i < pending; ++i)
      {
        if (pending_lfn[i].type == 0 && pending_lfn[i].checksum == want_checksum)
        {
          uint8_t lfn_del[32];
          std::memcpy(lfn_del, &pending_lfn[i], 32);
          lfn_del[0] = FAT_DIR_FREE;
          writer.WriteAt(pending_slot[i], lfn_del);
        }
      }
      found = true;
      break;
    }
    pending = 0;
    writer.NextSlot();
  }

  if (!found)
  {
    return ErrorCode::NOT_FOUND;
  }

  if (!is_dir)
  {
    const uint32_t start = (static_cast<uint32_t>(entry.clus_hi) << 16) | entry.clus_lo;
    if (start >= 2)
    {
      rc = ClusterFreeChain(start);
      if (rc != ErrorCode::OK)
      {
        return rc;
      }
    }
  }

  return SyncFsInfo();
}

ErrorCode Fat::RewriteDirEntry(uint32_t dir_clus, bool root16, const FatDirEntry& key,
                               const FatDirEntry& updated)
{
  DirWriter writer(*this, dir_clus, root16);
  auto rc = writer.Begin();
  if (rc != ErrorCode::OK)
  {
    return rc;
  }

  FatDirEntry scan;
  for (;;)
  {
    rc = writer.Advance();
    if (rc == ErrorCode::EMPTY)
    {
      return ErrorCode::NOT_FOUND;
    }
    if (rc != ErrorCode::OK)
    {
      return rc;
    }
    writer.Current(scan);
    if ((scan.attr & FAT_ATTR_LFN) == FAT_ATTR_LFN || scan.name[0] == FAT_DIR_FREE ||
        scan.name[0] == FAT_DIR_END)
    {
      writer.NextSlot();
      continue;
    }
    if (std::memcmp(scan.name, key.name, 11) == 0 && scan.size == key.size &&
        scan.clus_hi == key.clus_hi && scan.clus_lo == key.clus_lo)
    {
      Fat::DirSlot slot;
      writer.Slot(slot);
      return writer.WriteAt(slot, &updated);
    }
    writer.NextSlot();
  }
}

}  // namespace LibXR
