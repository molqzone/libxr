#pragma once

#include <cstddef>
#include <cstdint>

#include "fat.hpp"
#include "fat_structs.hpp"

namespace LibXR
{

/**
 * @class FatFile
 * @brief FAT 文件流式访问（M3a：只读子集）。
 *        FAT file streaming access (M3a: read-only subset).
 *
 * 构造即完成查找与打开；失败（不存在/是目录/介质错误）不 fail-fast，
 * 由 `IsValid()`/`GetOpenError()` 报告（可移动介质语义）。
 * Construction finds and opens the file; failure is reported through
 * IsValid()/GetOpenError() rather than failing fatally.
 */
class FatFile
{
 public:
  /**
   * @param fat 已挂载的 Fat / Mounted Fat instance
   * @param path 绝对文件路径 / Absolute file path
   */
  FatFile(Fat& fat, const char* path);

  /// 是否成功打开 / Whether the open succeeded.
  [[nodiscard]] bool IsValid() const { return open_err_ == ErrorCode::OK; }

  /// 打开错误码 / Open error code.
  [[nodiscard]] ErrorCode GetOpenError() const { return open_err_; }

  /// 文件大小（字节）/ File size in bytes.
  [[nodiscard]] uint64_t Size() const { return entry_.size; }

  /**
   * @brief 读数据 / Read data.
   * @param buf 目标缓冲区 / Destination buffer
   * @param len 输入=容量，输出=实读字节数 / In: capacity, out: bytes read
   */
  ErrorCode Read(RawData buf, size_t& len);

  /**
   * @brief 绝对定位 / Absolute seek.
   * @param offset 文件内偏移 / Offset within the file
   */
  ErrorCode Seek(uint64_t offset);

  /// 是否读到结尾 / Whether at end of file.
  [[nodiscard]] bool IsEof() const { return offset_ >= entry_.size; }

 private:
  /// 推进簇游标覆盖目标簇 / Advance the cluster cursor to cover a cluster index.
  ErrorCode AdvanceCursor(uint32_t target_clus_idx);

  /// 确保目标绝对偏移所在扇区在缓冲中 / Ensure the sector of an offset is buffered.
  ErrorCode EnsureSector(uint64_t abs_offset);

  Fat& fat_;
  FatDirEntry entry_;
  ErrorCode open_err_ = ErrorCode::OK;

  uint32_t start_clus_ = 0;    ///< 首簇 / First cluster
  uint32_t cur_clus_ = 0;      ///< 游标当前簇 / Cursor cluster
  uint32_t cur_clus_idx_ = 0;  ///< 游标簇序号（从 0）/ Cursor cluster index
  uint64_t offset_ = 0;        ///< 逻辑读偏移 / Logical offset

  bool sector_valid_ = false;   ///< 扇区缓冲有效 / Sector buffer valid
  uint64_t sector_offset_ = 0;  ///< 缓冲扇区的文件偏移 / File offset of buffer
  uint8_t sector_[512] = {};    ///< 扇区缓冲 / Sector buffer
};

}  // namespace LibXR
