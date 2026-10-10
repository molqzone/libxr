#pragma once

#include <cstdint>

namespace LibXR
{

/**
 * @brief FAT 盘上结构的只读内存镜像（全部 pack 到 1 字节对齐）。
 *        In-memory mirrors of the FAT on-disk structures (packed to 1 byte).
 *
 * 偏移依据 Microsoft FAT 规格与设计文档 §5。所有字段按盘上字节序解释，
 * 不假设主机字节序 / Offsets follow the FAT spec (design doc section 5).
 */
#pragma pack(push, 1)

/**
 * @struct FatBpb
 * @brief BIOS Parameter Block（boot sector 有效部分）。
 *        BIOS Parameter Block of the boot sector.
 */
struct FatBpb
{
  uint8_t jmp[3];        ///< 0   跳转指令（不校验）/ Jump instruction (not checked)
  uint8_t oem[8];        ///< 3   OEM 名 / OEM name
  uint16_t bytes_sec;    ///< 11  每扇区字节数 / Bytes per sector
  uint8_t sec_clus;      ///< 13  每簇扇区数 / Sectors per cluster
  uint16_t rsvd_sec;     ///< 14  保留扇区数 / Reserved sectors
  uint8_t num_fats;      ///< 16  FAT 表份数 / Number of FATs
  uint16_t root_ents;    ///< 17  根目录项数（FAT16）/ Root entries (FAT16)
  uint16_t tot_sec16;    ///< 19  总扇区数 16 位 / Total sectors (16-bit)
  uint8_t media;         ///< 21  媒体描述符 / Media descriptor
  uint16_t fat_sz16;     ///< 22  FAT 大小 16 位（FAT16）/ FAT size 16-bit (FAT16)
  uint16_t sec_per_trk;  ///< 24  每磁道扇区数（CHS，忽略）/ Sectors per track
  uint16_t num_heads;    ///< 26  磁头数（CHS，忽略）/ Number of heads
  uint32_t hidd_sec;     ///< 28  隐藏扇区数 / Hidden sectors
  uint32_t tot_sec32;    ///< 28[32] 总扇区数 32 位 / Total sectors (32-bit)

  // 以下字段偏移按实际 BPB 续排（FAT12/16 与 FAT32 分叉见设计文档 §5.1）。
  uint32_t fat_sz32;     ///< 36  FAT 大小 32 位（FAT32）/ FAT size 32-bit
  uint16_t ext_flags;    ///< 40  扩展标志（镜像信息）/ Ext flags (mirroring)
  uint16_t fs_ver;       ///< 42  文件系统版本 / FS version
  uint32_t root_clus;    ///< 44  根目录起始簇（FAT32）/ Root cluster
  uint16_t fs_info;      ///< 48  FSInfo 扇区号（FAT32）/ FSInfo sector
  uint16_t bk_boot_sec;  ///< 50  boot sector 备份 / Boot sector backup
  uint8_t reserved[12];  ///< 52  保留 / Reserved
  uint8_t drv_num;       ///< 64  驱动器号 / Drive number
  uint8_t reserved1;     ///< 65  保留 / Reserved
  uint8_t boot_sig;      ///< 66  扩展引导签名 0x29 / Ext boot signature
  uint32_t vol_id;       ///< 67  卷序列号 / Volume ID
  uint8_t vol_lab[11];   ///< 71  卷标 / Volume label
  uint8_t fs_type[8];    ///< 82  "FAT12/16/32" / File system type
};

static_assert(sizeof(FatBpb) == 90, "FatBpb layout mismatch");

/**
 * @brief boot sector 尾部签名偏移 / Boot sector tail signature offset.
 */
inline constexpr uint32_t FAT_BOOT_SIG_OFFSET = 510;

/**
 * @brief boot sector 合法签名值 / Valid boot sector signature value.
 */
inline constexpr uint16_t FAT_BOOT_SIG = 0xAA55;

/**
 * @struct FatDirEntry
 * @brief 32 字节目录项（短名）/ 32-byte short-name directory entry.
 */
struct FatDirEntry
{
  uint8_t name[11];     ///< 0  8.3 名 / 8.3 name
  uint8_t attr;         ///< 11 属性 / Attributes
  uint8_t nt_res;       ///< 12 NT 保留（大小写位）/ NT reserved (case bits)
  uint8_t crt_time_10;  ///< 13 创建时间十分之一 / Create time (10 ms units)
  uint16_t crt_time;    ///< 14 创建时间 / Create time
  uint16_t crt_date;    ///< 16 创建日期 / Create date
  uint16_t acc_date;    ///< 18 最后访问日期 / Last access date
  uint16_t clus_hi;     ///< 20 起始簇高 16 位（FAT32）/ Cluster high (FAT32)
  uint16_t wrt_time;    ///< 22 写入时间 / Write time
  uint16_t wrt_date;    ///< 24 写入日期 / Write date
  uint16_t clus_lo;     ///< 26 起始簇低 16 位 / Cluster low
  uint32_t size;        ///< 28 文件大小（目录为 0）/ File size (0 for dir)
};

static_assert(sizeof(FatDirEntry) == 32, "FatDirEntry layout mismatch");

/**
 * @struct FatLfnEntry
 * @brief 32 字节长文件名条目（attr == 0x0F）。
 *        32-byte long-file-name entry (attr == 0x0F).
 */
struct FatLfnEntry
{
  uint8_t seq;        ///< 0  序号；bit6(0x40)=末条 / Sequence; 0x40 marks last
  uint8_t name1[10];  ///< 1  字符 1-5（UCS-2LE）/ Chars 1-5
  uint8_t attr;       ///< 11 必须 0x0F / Must be 0x0F
  uint8_t type;       ///< 12 必须 0 / Must be 0
  uint8_t checksum;   ///< 13 短名校验和 / Short-name checksum
  uint8_t name2[12];  ///< 14 字符 6-11 / Chars 6-11
  uint16_t clus_lo;   ///< 26 必须 0 / Must be 0
  uint8_t name3[4];   ///< 28 字符 12-13 / Chars 12-13
};

static_assert(sizeof(FatLfnEntry) == 32, "FatLfnEntry layout mismatch");

#pragma pack(pop)

/// @name 目录项属性位 / Directory entry attribute bits
/// @{
inline constexpr uint8_t FAT_ATTR_RO = 0x01;       ///< 只读 / Read-only
inline constexpr uint8_t FAT_ATTR_HIDDEN = 0x02;   ///< 隐藏 / Hidden
inline constexpr uint8_t FAT_ATTR_SYSTEM = 0x04;   ///< 系统 / System
inline constexpr uint8_t FAT_ATTR_VOLUME = 0x08;   ///< 卷标 / Volume label
inline constexpr uint8_t FAT_ATTR_DIR = 0x10;      ///< 目录 / Directory
inline constexpr uint8_t FAT_ATTR_ARCHIVE = 0x20;  ///< 存档 / Archive
inline constexpr uint8_t FAT_ATTR_LFN = 0x0F;      ///< 长文件名条目 / LFN entry
/// @}

/// @name 目录项特殊首字节 / Special first bytes of a directory entry
/// @{
inline constexpr uint8_t FAT_DIR_FREE = 0xE5;   ///< 已删除 / Deleted
inline constexpr uint8_t FAT_DIR_END = 0x00;    ///< 目录结束（其后无项）/ End of dir
inline constexpr uint8_t FAT_DIR_THIS = 0x2E;   ///< "." 项 / "." entry
inline constexpr uint8_t FAT_DIR_KANJI = 0x05;  ///< 首字符实为 0xE5 的转义 / 0xE5 escape
/// @}

/// @name 簇链终结与坏簇值 / End-of-chain and bad-cluster markers
/// @{
inline constexpr uint16_t FAT16_EOC_MIN = 0xFFF8;      ///< FAT16 EOC 下界
inline constexpr uint16_t FAT16_BAD = 0xFFF7;          ///< FAT16 坏簇
inline constexpr uint32_t FAT32_EOC_MIN = 0x0FFFFFF8;  ///< FAT32 EOC 下界
inline constexpr uint32_t FAT32_BAD = 0x0FFFFFF7;      ///< FAT32 坏簇
/// @}

/// @name FAT 类型判定阈值（数据区簇数）/ Type thresholds (data cluster count)
/// @{
inline constexpr uint32_t FAT12_MAX_CLUSTERS = 4084;   ///< <4085 为 FAT12
inline constexpr uint32_t FAT16_MAX_CLUSTERS = 65524;  ///< <65525 为 FAT16
/// @}

}  // namespace LibXR
