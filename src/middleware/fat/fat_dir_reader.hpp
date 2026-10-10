#pragma once

#include <cstddef>
#include <cstdint>

#include "block_device.hpp"
#include "fat_structs.hpp"

namespace LibXR
{

class Fat;  // 前向声明：成员仅持引用 / Forward declaration: reference member only.

/**
 * @class FatDirReader
 * @brief FAT 目录流式读取引擎（内部细节，勿在应用层直接使用）。
 *        Directory streaming engine (internal detail; not for application use).
 *
 * 职责：沿簇链/连续区遍历目录扇区，解析 32 字节条目，组装 LFN 长名，
 * 回调完整条目。LFN 组装、校验和验证、短名回退只此一处实现。
 * Walks directory sectors, parses 32-byte entries and assembles long
 * names. This is the single place where LFN assembly/checksum/fallback
 * lives.
 */
class FatDirReader
{
 public:
  /**
   * @param fat 已挂载的 Fat / Mounted Fat instance
   * @param start_clus 目录起始簇（FAT16 根目录忽略）/ Start cluster
   * @param is_root16 是否 FAT16 根目录（连续区，非簇链）/ FAT16 root (flat)
   */
  FatDirReader(Fat& fat, uint32_t start_clus, bool is_root16);

  /**
   * @brief 读取下一条完整目录项 / Read the next complete directory entry.
   * @param name_out 文件名输出（M3a：ASCII；非 ASCII LFN 回退短名）
   *                 / Name output buffer
   * @param name_size 缓冲区大小 / Buffer size
   * @param entry_out 条目原始结构（attr/size/clus）/ Raw entry out
   * @return OK；目录结束返回 EMPTY；链断裂 CHECK_ERR
   */
  ErrorCode Next(char* name_out, size_t name_size, FatDirEntry& entry_out);

  /// 重置到目录开头 / Rewind to the beginning.
  void Rewind();

 private:
  /// 短名转可读形式（去空格、点插入）/ Render a short name.
  static void RenderShortName(const uint8_t name[11], char* out, size_t size);

  /// 计算短名校验和（LFN 算法）/ Short-name checksum (LFN algorithm).
  static uint8_t ShortNameChecksum(const uint8_t name[11]);

  /// 处理一条 LFN 片段 / Process an LFN fragment entry.
  void AccumulateLfn(const FatLfnEntry& lfn);

  /// 清空 LFN 累积状态 / Reset LFN accumulation state.
  void ResetLfn();

  /// 完成当前条目输出 / Finish the current entry.
  ErrorCode Emit(char* name_out, size_t name_size, FatDirEntry& entry_out);

  Fat& fat_;
  uint32_t start_clus_ = 0;
  bool is_root16_ = false;

  uint32_t cur_clus_ = 0;        ///< 当前簇 / Current cluster
  uint64_t sector_lba_ = 0;      ///< 当前扇区 LBA / Current sector LBA
  uint32_t sector_in_clus_ = 0;  ///< 簇内扇区索引 / Sector index within cluster
  bool started_ = false;         ///< 是否已开始 / Iteration started

  uint8_t sector_[512] = {};  ///< 扇区缓冲 / Sector buffer
  uint32_t entry_idx_ = 0;    ///< 扇区内条目索引（0..15）/ Entry index

  char lfn_buf_[256] = {};    ///< LFN 累积（UCS-2 解码后）/ Assembled LFN
  uint8_t lfn_seen_ = 0;      ///< 已见 LFN 片段数 / LFN fragments seen
  uint8_t lfn_checksum_ = 0;  ///< LFN 期望校验和 / Expected checksum
  bool lfn_ascii_ = true;     ///< 是否全 ASCII / All-ASCII flag
  uint8_t lfn_last_seq_ = 0;  ///< 末条序号 / Last sequence number
};

}  // namespace LibXR
