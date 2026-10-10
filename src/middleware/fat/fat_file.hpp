#pragma once

#include <cstddef>
#include <cstdint>

#include "fat.hpp"
#include "fat_structs.hpp"

namespace LibXR
{

/**
 * @class FatFile
 * @brief FAT 文件流式访问。
 *        FAT file streaming access.
 *
 * M3a 只读子集 + M3b 写入（创建/追加/截断）。
 * Read-only subset (M3a) plus write paths (M3b): create/append/truncate.
 *
 * 构造即完成查找/创建与打开；失败（不存在/是目录/介质错误）不 fail-fast，
 * 由 `IsValid()`/`GetOpenError()` 报告（可移动介质语义）。
 * Construction finds/creates and opens; failure is reported through
 * IsValid()/GetOpenError() rather than failing fatally.
 */
class FatFile
{
 public:
  /// 打开模式 / Open mode.
  enum class Mode : uint8_t
  {
    READ_ONLY = 0,     ///< 必须存在 / Must exist
    WRITE_NEW = 1,     ///< 存在则截断，不存在则创建 / Create or truncate
    WRITE_APPEND = 2,  ///< 打开并定位到末尾，不存在则创建 / Open at end, or create
    READ_WRITE = 3,    ///< 读写，不存在则创建 / Read-write, create if missing
  };

  /**
   * @param fat 已挂载的 Fat / Mounted Fat instance
   * @param path 绝对文件路径 / Absolute file path
   * @param mode 打开模式 / Open mode
   */
  FatFile(Fat& fat, const char* path, Mode mode = Mode::READ_ONLY);

  /// 是否成功打开 / Whether the open succeeded.
  [[nodiscard]] bool IsValid() const { return open_err_ == ErrorCode::OK; }

  /// 打开错误码 / Open error code.
  [[nodiscard]] ErrorCode GetOpenError() const { return open_err_; }

  /// 文件大小（字节，含未刷新的写入）/ Size in bytes (includes unflushed writes).
  [[nodiscard]] uint64_t Size() const { return entry_.size; }

  /**
   * @brief 读数据 / Read data.
   * @param buf 目标缓冲区 / Destination buffer
   * @param len 输入=容量，输出=实读字节数 / In: capacity, out: bytes read
   */
  ErrorCode Read(RawData buf, size_t& len);

  /**
   * @brief 写数据 / Write data.
   * @param buf 源缓冲区 / Source buffer
   * @param len 输入=待写字节数，输出=实写字节数 / In: length, out: written
   */
  ErrorCode Write(ConstRawData buf, size_t& len);

  /**
   * @brief 绝对定位 / Absolute seek.
   * @param offset 文件内偏移 / Offset within the file
   */
  ErrorCode Seek(uint64_t offset);

  /**
   * @brief 截断到指定大小 / Truncate to a size.
   * @param size 目标大小 / Target size
   */
  ErrorCode Truncate(uint64_t size);

  /**
   * @brief 刷盘：数据已直写，此处同步 FAT/目录项/FSInfo。
   *        Flush: data is written through; sync FAT/dir entry/FSInfo.
   */
  ErrorCode Sync();

  /// 是否读到结尾 / Whether at end of file.
  [[nodiscard]] bool IsEof() const { return offset_ >= entry_.size; }

 private:
  /// 推进簇游标覆盖目标簇 / Advance the cluster cursor to cover a cluster index.
  ErrorCode AdvanceCursor(uint32_t target_clus_idx);

  /// 确保目标绝对偏移所在扇区在缓冲中 / Ensure the sector of an offset is buffered.
  ErrorCode EnsureSector(uint64_t abs_offset);

  /// 为写入确保簇覆盖（必要时分配新簇）/ Ensure cluster coverage for writes.
  ErrorCode EnsureClusters(uint32_t clus_idx_end);

  /// 写当前缓冲扇区 / Write the buffered sector through.
  ErrorCode FlushSector();

  /// 更新目录项（size/起始簇）/ Update the directory entry (size/start cluster).
  ErrorCode UpdateDirEntry();

  Fat& fat_;
  FatDirEntry entry_;
  ErrorCode open_err_ = ErrorCode::OK;
  Mode mode_ = Mode::READ_ONLY;

  uint32_t parent_clus_ = 0;    ///< 父目录起始簇 / Parent directory cluster
  bool parent_root16_ = false;  ///< 父目录是否 FAT16 根 / Parent is FAT16 root

  FatDirEntry disk_entry_;  ///< 最近一次落盘的目录项（Sync 匹配键）/ Last synced
                            ///< directory entry (Sync key)

  uint32_t start_clus_ = 0;    ///< 首簇 / First cluster
  uint32_t cur_clus_ = 0;      ///< 游标当前簇 / Cursor cluster
  uint32_t cur_clus_idx_ = 0;  ///< 游标簇序号（从 0）/ Cursor cluster index
  uint64_t offset_ = 0;        ///< 逻辑偏移 / Logical offset

  bool sector_valid_ = false;   ///< 扇区缓冲有效 / Sector buffer valid
  bool sector_dirty_ = false;   ///< 扇区缓冲已改（写路径）/ Buffer dirty (write path)
  uint64_t sector_offset_ = 0;  ///< 缓冲扇区的文件偏移 / File offset of buffer
  uint8_t sector_[512] = {};    ///< 扇区缓冲 / Sector buffer
};

}  // namespace LibXR
