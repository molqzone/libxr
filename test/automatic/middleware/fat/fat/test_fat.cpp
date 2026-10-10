/**
 * @file test_fat.cpp
 * @brief FAT 只读 host 差分测试（M3a 验收）。
 *        FAT read-only host differential tests (M3a acceptance).
 *
 * oracle：mtools（mkfs.vfat/mcopy）是独立 FAT 实现——它写盘，我们读；
 * 内容必须逐字节一致。/ mtools is an independent FAT implementation: it
 * writes, we read, and the bytes must match exactly.
 *
 * 依赖 dosfstools 的 mkfs.vfat/mcopy/mmd；缺失时大声跳过（CI 必须装）。
 * Requires mkfs.vfat/mcopy/mmd from dosfstools; skips loudly when missing.
 */

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "block_device.hpp"
#include "fat.hpp"
#include "fat_dir.hpp"
#include "fat_file.hpp"
#include "libxr_def.hpp"
#include "test_assert.hpp"

namespace
{

/**
 * @brief host 文件后端块设备（差分测试专用）。
 *        Host file-backed block device (differential testing).
 */
class FileBlockDevice : public LibXR::BlockDevice
{
 public:
  explicit FileBlockDevice(const char* path)
  {
    read_ = std::fopen(path, "rb");
    write_ = read_;  // 只读测试也统一以 r+b 打开 / Open r+b for uniformity
    if (read_ == nullptr)
    {
      return;
    }
    std::fseek(read_, 0, SEEK_END);
    sectors_ = static_cast<uint64_t>(std::ftell(read_)) / 512;
    std::fseek(read_, 0, SEEK_SET);
  }

  ~FileBlockDevice() override
  {
    if (read_ != nullptr)
    {
      std::fclose(read_);
    }
  }

  [[nodiscard]] uint32_t SectorSize() const override { return 512; }
  [[nodiscard]] uint64_t SectorCount() const override { return sectors_; }

  LibXR::ErrorCode Read(uint64_t lba, uint32_t count, LibXR::RawData buf) override
  {
    if (read_ == nullptr)
    {
      return LibXR::ErrorCode::FAILED;
    }
    if (buf.size_ < static_cast<size_t>(count) * 512)
    {
      return LibXR::ErrorCode::ARG_ERR;
    }
    std::fseek(read_, static_cast<long>(lba) * 512, SEEK_SET);
    if (std::fread(buf.addr_, 512, count, read_) != count)
    {
      return LibXR::ErrorCode::FAILED;
    }
    return LibXR::ErrorCode::OK;
  }

  LibXR::ErrorCode Write(uint64_t, uint32_t, LibXR::ConstRawData) override
  {
    return LibXR::ErrorCode::NOT_SUPPORT;
  }

  LibXR::ErrorCode Trim(uint64_t, uint32_t) override
  {
    return LibXR::ErrorCode::NOT_SUPPORT;
  }

  LibXR::ErrorCode Sync() override { return LibXR::ErrorCode::OK; }

 private:
  std::FILE* read_ = nullptr;
  std::FILE* write_ = nullptr;
  uint64_t sectors_ = 0;
};

bool ToolExists(const char* name)
{
  std::string cmd = std::string("command -v ") + name + " >/dev/null 2>&1";
  return std::system(cmd.c_str()) == 0;
}

std::string RunCommand(const std::string& cmd)
{
  std::string out;
  if (std::FILE* pipe = popen((cmd + " 2>&1").c_str(), "r"))
  {
    char buf[256];
    while (std::fgets(buf, sizeof(buf), pipe) != nullptr)
    {
      out += buf;
    }
    pclose(pipe);
  }
  return out;
}

struct Entry
{
  std::string name;
  bool is_dir;
  uint64_t size;
};

std::string ReadAll(LibXR::FatFile& file)
{
  std::string out;
  uint8_t buf[128];
  for (;;)
  {
    size_t len = sizeof(buf);
    auto rc = file.Read({buf, sizeof(buf)}, len);
    if (rc != LibXR::ErrorCode::OK)
    {
      return out;
    }
    if (len == 0)
    {
      return out;
    }
    out.append(reinterpret_cast<char*>(buf), len);
  }
}

std::string ToLower(std::string s)
{
  for (auto& c : s)
  {
    if (c >= 'A' && c <= 'Z')
    {
      c = static_cast<char>(c + 32);
    }
  }
  return s;
}

void TestOneImage(const char* img, const char* fat_flag, bool expect_fat32)
{
  // mkfs.vfat -C 不清除已有镜像；残留文件会让 mcopy 弹交互提示并阻塞。
  // mkfs.vfat -C does not wipe an existing image; stale files make mcopy
  // prompt on stdin and block.
  RunCommand(std::string("rm -f ") + img);
  std::string mk = std::string("mkfs.vfat -F ") + fat_flag + " -C " + img + " ";
  mk += (std::string(fat_flag) == "16") ? "16384" : "32768";
  if (RunCommand(mk).find("mkfs.fat") == std::string::npos)
  {
    std::fprintf(stderr, "FAT test: mkfs.vfat failed for %s\n", img);
  }

  const char* kHello = "hello fat world\n";
  const char* kConfig = "log_level=4\nname=openocd\n";
  const char* kNested = "nested-content\n";

  RunCommand(std::string("echo -n '") + kHello + "' > /tmp/opencode/fat_hello.txt");
  RunCommand(std::string("echo -n '") + kConfig + "' > /tmp/opencode/fat_config.txt");
  RunCommand(std::string("echo -n '") + kNested + "' > /tmp/opencode/fat_nested.txt");
  RunCommand(std::string("mcopy -i ") + img +
             " /tmp/opencode/fat_hello.txt ::/hello.txt");
  RunCommand(std::string("mcopy -i ") + img +
             " /tmp/opencode/fat_config.txt ::/config.txt");
  RunCommand(std::string("mmd -i ") + img + " ::/sub");
  RunCommand(std::string("mcopy -i ") + img +
             " /tmp/opencode/fat_nested.txt ::/sub/nested.txt");

  FileBlockDevice dev(img);
  TEST_ASSERT(dev.SectorCount() != 0);

  LibXR::Fat fat(dev);
  TEST_ASSERT(fat.IsMounted());
  TEST_ASSERT(fat.GetType() ==
              (expect_fat32 ? LibXR::Fat::Type::FAT32 : LibXR::Fat::Type::FAT16));

  LibXR::FatDir dir(fat, "/");
  std::vector<Entry> entries;
  char name[256];
  bool is_dir = false;
  uint64_t size = 0;
  while (dir.Next(name, sizeof(name), is_dir, size) == LibXR::ErrorCode::OK)
  {
    entries.push_back({name, is_dir, size});
  }
  bool hello_ok = false;
  bool config_ok = false;
  bool sub_ok = false;
  for (const auto& e : entries)
  {
    // mtools 对 8.3 兼容名只写短名（大写）；FAT 语义大小写不敏感。
    // mtools stores only uppercase short names for 8.3-compatible names.
    const std::string lower = ToLower(e.name);
    if (lower == "hello.txt" && !e.is_dir && e.size == std::strlen(kHello))
    {
      hello_ok = true;
    }
    if (lower == "config.txt" && !e.is_dir && e.size == std::strlen(kConfig))
    {
      config_ok = true;
    }
    if (lower == "sub" && e.is_dir)
    {
      sub_ok = true;
    }
  }
  TEST_ASSERT(hello_ok);
  TEST_ASSERT(config_ok);
  TEST_ASSERT(sub_ok);

  LibXR::FatFile hello(fat, "/hello.txt");
  TEST_ASSERT(hello.IsValid());
  TEST_ASSERT(hello.Size() == std::strlen(kHello));
  TEST_ASSERT(ReadAll(hello) == kHello);

  LibXR::FatFile config(fat, "/config.txt");
  TEST_ASSERT(config.IsValid());
  TEST_ASSERT(ReadAll(config) == kConfig);

  LibXR::FatFile nested(fat, "/sub/nested.txt");
  TEST_ASSERT(nested.IsValid());
  TEST_ASSERT(ReadAll(nested) == kNested);

  LibXR::FatFile seek_test(fat, "/config.txt");
  TEST_ASSERT(seek_test.IsValid());
  TEST_ASSERT(seek_test.Seek(10) == LibXR::ErrorCode::OK);
  char piece[32] = {};
  size_t len = sizeof(piece);
  TEST_ASSERT(seek_test.Read({piece, sizeof(piece) - 1}, len) == LibXR::ErrorCode::OK);
  TEST_ASSERT(len == std::strlen(kConfig) - 10);
  TEST_ASSERT(std::memcmp(piece, kConfig + 10, len) == 0);

  LibXR::FatFile missing(fat, "/nope.txt");
  TEST_ASSERT(!missing.IsValid());
  TEST_ASSERT(missing.GetOpenError() == LibXR::ErrorCode::NOT_FOUND);

  LibXR::FatFile wrong_dir(fat, "/sub");
  TEST_ASSERT(!wrong_dir.IsValid());

  LibXR::FatDir bad_dir(fat, "/no_such_dir");
  TEST_ASSERT(bad_dir.Next(name, sizeof(name), is_dir, size) ==
              LibXR::ErrorCode::NOT_FOUND);
}

}  // namespace

void test_fat()
{
  if (!ToolExists("mkfs.vfat") || !ToolExists("mcopy") || !ToolExists("mmd"))
  {
    std::fprintf(stderr,
                 "FAT test: dosfstools (mkfs.vfat/mcopy/mmd) missing — SKIPPED. "
                 "Install dosfstools to run the M3a oracle tests.\n");
    return;
  }

  const char* kFat16Img = "/tmp/opencode/fat16.img";
  const char* kFat32Img = "/tmp/opencode/fat32.img";

  TestOneImage(kFat16Img, "16", false);
  TestOneImage(kFat32Img, "32", true);

  std::printf("FAT M3a tests passed (FAT16 + FAT32)\n");
}
