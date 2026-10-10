#include "fat_dir.hpp"

namespace LibXR
{

FatDir::Target FatDir::Resolve(Fat& fat, const char* path)
{
  Target target{0, false, ErrorCode::OK};
  target.err = fat.ResolveDir(path, target.clus, target.root16);
  return target;
}

FatDir::FatDir(Fat& fat, const char* path)
    : target_(Resolve(fat, path)), reader_(fat, target_.clus, target_.root16)
{
}

ErrorCode FatDir::Next(char* name_out, size_t name_size, bool& is_dir,
                       uint64_t& file_size)
{
  if (target_.err != ErrorCode::OK)
  {
    return target_.err;
  }

  FatDirEntry entry;
  auto rc = reader_.Next(name_out, name_size, entry);
  if (rc != ErrorCode::OK)
  {
    return rc;
  }

  is_dir = (entry.attr & FAT_ATTR_DIR) != 0;
  file_size = entry.size;
  return ErrorCode::OK;
}

}  // namespace LibXR
