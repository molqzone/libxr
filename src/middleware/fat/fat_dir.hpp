#pragma once

#include <cstddef>
#include <cstdint>

#include "fat.hpp"
#include "fat_dir_reader.hpp"

namespace LibXR
{

/**
 * @class FatDir
 * @brief FAT 目录枚举（只读）。
 *        FAT directory enumeration (read-only).
 *
 * 构造即完成路径解析与目录打开；解析失败时 Next 返回解析错误。
 * Construction resolves the path and opens the directory; Next returns the
 * resolution error when it failed.
 */
class FatDir
{
 public:
  /**
   * @param fat 已挂载的 Fat / Mounted Fat instance
   * @param path 绝对路径；"/" 为根 / Absolute path; "/" is the root
   */
  FatDir(Fat& fat, const char* path);

  /**
   * @brief 取下一条目录项 / Get the next directory entry.
   * @param name_out 文件名缓冲区 / Name buffer
   * @param name_size 缓冲区大小 / Buffer size
   * @param is_dir 输出：是否目录 / Out: is directory
   * @param file_size 输出：文件大小（目录为 0）/ Out: size (0 for dirs)
   * @return OK；目录结束返回 EMPTY / OK; EMPTY at end
   */
  ErrorCode Next(char* name_out, size_t name_size, bool& is_dir, uint64_t& file_size);

  /// 重置到目录开头 / Rewind to the beginning.
  void Rewind() { reader_.Rewind(); }

 private:
  struct Target
  {
    uint32_t clus;  ///< 目录起始簇 / Start cluster
    bool root16;    ///< 是否 FAT16 根目录 / FAT16 root (flat)
    ErrorCode err;  ///< 路径解析结果 / Path resolution result
  };

  static Target Resolve(Fat& fat, const char* path);

  Target target_;
  FatDirReader reader_;
};

}  // namespace LibXR
