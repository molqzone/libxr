#include "fat_dir_reader.hpp"

#include <cstring>

#include "fat.hpp"

namespace LibXR
{

FatDirReader::FatDirReader(Fat& fat, uint32_t start_clus, bool is_root16)
    : fat_(fat), start_clus_(start_clus), is_root16_(is_root16)
{
  Rewind();
}

void FatDirReader::Rewind()
{
  cur_clus_ = start_clus_;
  sector_in_clus_ = 0;
  sector_lba_ = 0;
  started_ = false;
  entry_idx_ = 0;

  std::memset(lfn_buf_, 0, sizeof(lfn_buf_));
  lfn_seen_ = 0;
  lfn_checksum_ = 0;
  lfn_ascii_ = true;
  lfn_last_seq_ = 0;
}

void FatDirReader::RenderShortName(const uint8_t name[11], char* out, size_t size)
{
  size_t pos = 0;

  auto put = [&](char c)
  {
    if (pos + 1 < size)
    {
      out[pos++] = c;
    }
  };

  for (size_t i = 0; i < 8; ++i)
  {
    const uint8_t c = (i == 0 && name[0] == FAT_DIR_KANJI) ? FAT_DIR_FREE : name[i];
    if (c == ' ')
    {
      break;
    }
    put(static_cast<char>(c));
  }

  bool has_ext = false;
  for (size_t i = 8; i < 11; ++i)
  {
    if (name[i] != ' ')
    {
      has_ext = true;
      break;
    }
  }
  if (has_ext)
  {
    put('.');
    for (size_t i = 8; i < 11; ++i)
    {
      if (name[i] == ' ')
      {
        break;
      }
      put(static_cast<char>(name[i]));
    }
  }

  if (size > 0)
  {
    out[pos] = '\0';
  }
}

uint8_t FatDirReader::ShortNameChecksum(const uint8_t name[11])
{
  uint8_t sum = 0;
  for (size_t i = 0; i < 11; ++i)
  {
    sum = static_cast<uint8_t>(((sum & 1) << 7) + (sum >> 1) + name[i]);
  }
  return sum;
}

void FatDirReader::AccumulateLfn(const FatLfnEntry& lfn)
{
  const uint8_t seq = lfn.seq & 0x1F;
  if (seq == 0 || seq > 20)
  {
    return;  // 非法序号 / Illegal sequence
  }

  if (lfn.seq & 0x40)
  {
    lfn_last_seq_ = seq;
    std::memset(lfn_buf_, 0, sizeof(lfn_buf_));
    lfn_ascii_ = true;
  }
  else if (lfn_seen_ == 0)
  {
    return;  // 未见末条（物理首条）即收到中段 / Middle without head
  }

  lfn_checksum_ = lfn.checksum;
  lfn_seen_++;

  const uint8_t* chars[3] = {lfn.name1, lfn.name2, lfn.name3};
  const size_t lens[3] = {5, 6, 2};
  size_t slot = (seq - 1) * 13;

  for (size_t part = 0; part < 3; ++part)
  {
    for (size_t i = 0; i < lens[part]; ++i)
    {
      uint16_t c = 0;
      std::memcpy(&c, chars[part] + i * 2, 2);
      if (c == 0)
      {
        return;  // 名结束 / End of name
      }
      if (c > 0x7F)
      {
        lfn_ascii_ = false;
      }
      if (slot < sizeof(lfn_buf_))
      {
        lfn_buf_[slot++] = static_cast<char>(c & 0xFF);
      }
    }
  }
}

void FatDirReader::ResetLfn()
{
  std::memset(lfn_buf_, 0, sizeof(lfn_buf_));
  lfn_seen_ = 0;
  lfn_checksum_ = 0;
  lfn_ascii_ = true;
  lfn_last_seq_ = 0;
}

ErrorCode FatDirReader::Emit(char* name_out, size_t name_size, FatDirEntry& entry_out)
{
  const bool use_lfn = lfn_seen_ > 0 && lfn_ascii_ && lfn_last_seq_ > 0 &&
                       lfn_seen_ >= lfn_last_seq_ &&
                       lfn_checksum_ == ShortNameChecksum(entry_out.name);

  if (use_lfn)
  {
    const size_t len = std::strlen(lfn_buf_);
    if (len >= name_size)
    {
      return ErrorCode::SIZE_ERR;
    }
    std::memcpy(name_out, lfn_buf_, len + 1);
  }
  else
  {
    RenderShortName(entry_out.name, name_out, name_size);
  }

  ResetLfn();
  return ErrorCode::OK;
}

ErrorCode FatDirReader::Next(char* name_out, size_t name_size, FatDirEntry& entry_out)
{
  const uint32_t entries_per_sector = fat_.BytesPerSector() / 32;
  const uint32_t sec_per_clus = fat_.SectorsPerCluster();

  for (;;)
  {
    if (!started_ || entry_idx_ >= entries_per_sector)
    {
      entry_idx_ = 0;

      if (is_root16_)
      {
        if (sector_in_clus_ >= fat_.RootSectorCount())
        {
          return ErrorCode::EMPTY;
        }
        sector_lba_ = fat_.RootStartSector() + sector_in_clus_;
        sector_in_clus_++;
      }
      else
      {
        if (started_ && sector_in_clus_ >= sec_per_clus)
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

    FatDirEntry entry;
    std::memcpy(&entry, sector_ + entry_idx_ * 32, 32);
    entry_idx_++;

    if (entry.name[0] == FAT_DIR_END)
    {
      return ErrorCode::EMPTY;
    }
    if (entry.name[0] == FAT_DIR_FREE)
    {
      ResetLfn();  // 孤儿 LFN / Orphan LFN
      continue;
    }
    // "." 与 ".." 对枚举无意义，跳过（设计文档 §6.7）/ Skip dot entries (doc 6.7)
    if (entry.name[0] == FAT_DIR_THIS &&
        (entry.name[1] == ' ' || entry.name[1] == FAT_DIR_THIS))
    {
      continue;
    }
    if ((entry.attr & FAT_ATTR_LFN) == FAT_ATTR_LFN)
    {
      FatLfnEntry lfn;
      std::memcpy(&lfn, &entry, 32);
      AccumulateLfn(lfn);
      continue;
    }
    if (entry.attr & FAT_ATTR_VOLUME)
    {
      continue;  // 卷标 / Volume label
    }

    entry_out = entry;
    return Emit(name_out, name_size, entry);
  }
}

}  // namespace LibXR
