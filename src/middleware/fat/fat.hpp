#pragma once

#include <cstddef>
#include <cstdint>

#include "block_device.hpp"
#include "fat_structs.hpp"

namespace LibXR
{

/**
 * @class Fat
 * @brief FAT16/FAT32 只读文件系统核心（M3a：挂载 + 枚举 + 文件读）。
 *        FAT16/FAT32 read-only filesystem core (M3a: mount, enumerate, read).
 *
 * 构造即完成挂载；失败（含无卡/坏盘）不 fail-fast——可移动介质语义同
 * `ESP32SDMMCCard`：`IsMounted()` 为 false，访问返回错误码。
 * Mounting happens in the constructor; failure is not fatal (removable
 * media semantics): IsMounted() reports it and access returns errors.
 *
 * @note 不持有整张 FAT 表：按需读扇区 + 单扇区缓存（设计文档 §6.2）。
 */
class Fat
{
 public:
  /// FAT 类型 / FAT type.
  enum class Type : uint8_t
  {
    FAT16,  ///< FAT16
    FAT32,  ///< FAT32
  };

  /**
   * @brief 构造并挂载 / Construct and mount.
   * @param dev 块设备（生命周期须长于本对象）/ Block device (must outlive)
   */
  explicit Fat(BlockDevice& dev);

  /// 是否已挂载 / Whether mounted.
  [[nodiscard]] bool IsMounted() const { return mounted_; }

  /// 挂载错误码：0=成功，否则为 ErrorCode / Mount error code (0 on success).
  [[nodiscard]] int GetMountError() const { return mount_err_; }

  /// 文件系统类型 / Filesystem type.
  [[nodiscard]] Type GetType() const { return static_cast<Type>(type_); }

  /// 每簇字节数 / Bytes per cluster.
  [[nodiscard]] uint32_t ClusterSize() const { return sec_clus_ * bytes_sec_; }

  /**
   * @brief 打开根目录或子目录（M3a：只读枚举入口）。
   *        Open the root or a subdirectory for read-only enumeration.
   * @param path 绝对路径；"/" 为根 / Absolute path; "/" is the root
   */
  ErrorCode OpenRoot(uint32_t& start_clus, bool& is_root16);

  /**
   * @brief 解析绝对路径到目录（供 FatDir/FatFile 使用）。
   *        Resolve an absolute path to a directory (for FatDir/FatFile).
   * @param path 绝对路径；"/" 为根；组件名大小写不敏感（短名规则）
   * @param start_clus 输出：目录起始簇 / Out: start cluster
   * @param is_root16 输出：是否 FAT16 根目录 / Out: FAT16 root (flat)
   * @return OK / NOT_FOUND（路径中某级不存在）/ ARG_ERR（非绝对路径等）
   */
  ErrorCode ResolveDir(const char* path, uint32_t& start_clus, bool& is_root16);

  /**
   * @brief 按路径查找文件（供 FatFile 使用）。
   *        Find a file by path (for FatFile).
   * @param path 绝对文件路径 / Absolute file path
   * @param entry_out 输出：目录项 / Out: directory entry
   * @return OK / NOT_FOUND（不存在或是目录）/ ARG_ERR / STATE_ERR
   */
  ErrorCode FindFile(const char* path, FatDirEntry& entry_out);

  // ---- 内部服务（FatFile/FatDir 使用，非公开 API）----

  /// 读一个扇区到调用方缓冲 / Read one sector into a caller buffer.
  ErrorCode ReadSector(uint64_t lba, RawData buf);

  /// 取簇链下一簇；EOC 时 next == 0xFFFFFFFF / Next cluster in chain.
  ErrorCode NextCluster(uint32_t clus, uint32_t& next);

  /// 簇号转首扇区 / First sector of a cluster.
  [[nodiscard]] uint64_t ClusterToSector(uint32_t clus) const
  {
    return data_start_ + (static_cast<uint64_t>(clus) - 2) * sec_clus_;
  }

  /// 是否簇链终点 / Whether a FAT entry marks end-of-chain.
  [[nodiscard]] bool IsEoc(uint32_t value) const
  {
    return (type_ == TYPE_FAT32) ? (value >= FAT32_EOC_MIN) : (value >= FAT16_EOC_MIN);
  }

  /// 是否坏簇 / Whether a FAT entry marks a bad cluster.
  [[nodiscard]] bool IsBadCluster(uint32_t value) const
  {
    return (type_ == TYPE_FAT32) ? (value == FAT32_BAD) : (value == FAT16_BAD);
  }

  /// 数据区簇总数 / Total data-area cluster count.
  [[nodiscard]] uint32_t TotalClusters() const { return total_clusters_; }

  /// 每扇区字节数 / Bytes per sector.
  [[nodiscard]] uint32_t BytesPerSector() const { return bytes_sec_; }

  /// 每簇扇区数 / Sectors per cluster.
  [[nodiscard]] uint32_t SectorsPerCluster() const { return sec_clus_; }

  /// FAT16 根目录起始扇区 / FAT16 root start sector.
  [[nodiscard]] uint64_t RootStartSector() const { return root_start_; }

  /// FAT16 根目录扇区数（FAT32 为 0）/ FAT16 root sector count (0 for FAT32).
  [[nodiscard]] uint32_t RootSectorCount() const { return root_secs_; }

 private:
  static constexpr uint8_t TYPE_FAT16 = 0;
  static constexpr uint8_t TYPE_FAT32 = 1;

  /// 挂载内部流程（失败返回非 OK）/ Mount internal steps.
  ErrorCode Mount();

  /// 使 FAT 扇区缓存失效 / Invalidate the FAT sector cache.
  void InvalidateFatCache()
  {
    fat_cache_valid_ = false;
    fat_cache_lba_ = UINT64_MAX;
  }

  /// 读 FAT 项（走单扇区缓存）/ Read a FAT entry (single-sector cache).
  ErrorCode ReadFatEntry(uint32_t clus, uint32_t& value);

  BlockDevice& dev_;
  bool mounted_ = false;
  int mount_err_ = 0;
  uint8_t type_ = TYPE_FAT16;

  uint32_t bytes_sec_ = 512;     ///< 每扇区字节 / Bytes per sector
  uint32_t sec_clus_ = 0;        ///< 每簇扇区 / Sectors per cluster
  uint32_t num_fats_ = 0;        ///< FAT 份数 / FAT copies
  uint32_t fat_sz_ = 0;          ///< 每份 FAT 扇区数 / Sectors per FAT
  uint32_t root_ents_ = 0;       ///< FAT16 根目录项数 / FAT16 root entries
  uint32_t root_secs_ = 0;       ///< FAT16 根目录扇区数；FAT32 为 0 / Root sectors
  uint64_t fat_start_ = 0;       ///< FAT 区起始扇区 / FAT start sector
  uint64_t root_start_ = 0;      ///< FAT16 根目录起始扇区 / FAT16 root start
  uint64_t data_start_ = 0;      ///< 数据区起始扇区 / Data start sector
  uint64_t total_secs_ = 0;      ///< 总扇区数 / Total sectors
  uint32_t total_clusters_ = 0;  ///< 数据区簇数 / Data clusters
  uint32_t root_clus_ = 0;       ///< FAT32 根簇 / FAT32 root cluster

  bool fat_cache_valid_ = false;       ///< FAT 缓存有效 / FAT cache valid
  uint64_t fat_cache_lba_ = 0;         ///< 缓存扇区号 / Cached LBA
  uint32_t fat_cache_data_[128] = {};  ///< 缓存内容（512B）/ Cache content
};

}  // namespace LibXR
