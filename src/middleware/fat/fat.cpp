#include "fat.hpp"

#include <cstring>

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

}  // namespace LibXR
