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
    read_ = std::fopen(path, "r+b");
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

  LibXR::ErrorCode Write(uint64_t lba, uint32_t count, LibXR::ConstRawData buf) override
  {
    if (buf.size_ < static_cast<size_t>(count) * 512)
    {
      return LibXR::ErrorCode::ARG_ERR;
    }
    std::fseek(write_, static_cast<long>(lba) * 512, SEEK_SET);
    if (std::fwrite(buf.addr_, 512, count, write_) != count)
    {
      return LibXR::ErrorCode::FAILED;
    }
    std::fflush(write_);
    return LibXR::ErrorCode::OK;
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
  // setsid：子进程无控制终端，mtools 的 /dev/tty 提示直接失败并按默认处理
  // （stdin 已是 /dev/null）。PTY 下不会阻塞。
  // setsid: children get no controlling terminal, so mtools' /dev/tty prompt
  // fails and falls back to the (already /dev/null) stdin instead of blocking.
  if (std::FILE* pipe = popen(("setsid " + cmd + " 2>&1 </dev/null").c_str(), "r"))
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

int RunCommandRc(const std::string& cmd)
{
  std::FILE* pipe = popen(("setsid " + cmd + " 2>&1 </dev/null").c_str(), "r");
  if (pipe == nullptr)
  {
    return -1;
  }
  char buf[256];
  while (std::fgets(buf, sizeof(buf), pipe) != nullptr)
  {
  }
  const int rc = pclose(pipe);
  return (rc == -1) ? -1 : (rc >> 8);  // WEXITSTATUS
}

bool ReadHostFile(const char* path, std::string& out)
{
  std::FILE* f = std::fopen(path, "rb");
  if (f == nullptr)
  {
    return false;
  }
  char buf[256];
  size_t n = 0;
  while ((n = std::fread(buf, 1, sizeof(buf), f)) > 0)
  {
    out.append(buf, n);
  }
  std::fclose(f);
  return true;
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
  // FAT32 需 ≥65525 簇才合规（否则 fsck 对格式本身报 warning）。
  // FAT32 requires >=65525 clusters to be spec-valid (fsck warns otherwise).
  mk += (std::string(fat_flag) == "16") ? "16384" : "69632";
  if (RunCommand(mk).find("mkfs.fat") == std::string::npos)
  {
    std::fprintf(stderr, "FAT test: mkfs.vfat failed for %s\n", img);
  }

  const char* kHello = "hello fat world\n";
  const char* kConfig = "log_level=4\nname=openocd\n";
  const char* kNested = "nested-content\n";

  RunCommand(std::string("echo -n '") + kHello + "' > " FAT_TEST_DIR "hello.txt");
  RunCommand(std::string("echo -n '") + kConfig + "' > " FAT_TEST_DIR "config.txt");
  RunCommand(std::string("echo -n '") + kNested + "' > " FAT_TEST_DIR "nested.txt");
  RunCommand(std::string("mcopy -i ") + img + " " + std::string(FAT_TEST_DIR) +
             "hello.txt ::/hello.txt");
  RunCommand(std::string("mcopy -i ") + img + " " + std::string(FAT_TEST_DIR) +
             "config.txt ::/config.txt");
  RunCommand(std::string("mmd -i ") + img + " ::/sub");
  RunCommand(std::string("mcopy -i ") + img + " " + std::string(FAT_TEST_DIR) +
             "nested.txt ::/sub/nested.txt");

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

  // ---- M3b: 写路径 + oracle 差分（fsck.vfat -n 零告警 + mtools 读回）----
  const char* kNew = "written-by-libxr-fat\n";
  {
    LibXR::FatFile f(fat, "/newfile.txt", LibXR::FatFile::Mode::WRITE_NEW);
    TEST_ASSERT(f.IsValid());
    const LibXR::ConstRawData w = {kNew, std::strlen(kNew)};
    size_t len = std::strlen(kNew);
    TEST_ASSERT(f.Write(w, len) == LibXR::ErrorCode::OK);
    TEST_ASSERT(len == std::strlen(kNew));
    TEST_ASSERT(f.Sync() == LibXR::ErrorCode::OK);
    TEST_ASSERT(f.Size() == std::strlen(kNew));
  }

  // fsck.vfat -n：写后文件系统必须零告警（接受标准）/ fsck must report clean.
  {
    const int rc = RunCommandRc(std::string("fsck.vfat -n ") + img);
    if (rc != 0)
    {
      const std::string detail = RunCommand(std::string("fsck.vfat -n -v ") + img);
      std::fprintf(stderr, "DBG fsck output:\n%s\n", detail.c_str());
    }
    TEST_ASSERT(rc == 0);
  }

  // mtools 独立读回 / Independent readback via mtools.
  {
    const std::string rb = std::string(FAT_TEST_DIR "readback.txt");
    RunCommand(std::string("rm -f ") + rb);
    const std::string mcopy_out =
        RunCommand(std::string("mcopy -i ") + img + " ::/newfile.txt " + rb);
    if (!mcopy_out.empty())
    {
      std::fprintf(stderr, "DBG mcopy out: %s\n", mcopy_out.c_str());
    }
    const int rc = RunCommandRc(std::string("mcopy -i ") + img + " ::/newfile.txt " + rb);
    TEST_ASSERT(rc == 0);
    std::string content;
    TEST_ASSERT(ReadHostFile(rb.c_str(), content));
    if (content != kNew)
    {
      std::fprintf(stderr, "DBG readback len=%zu: '%s'\n", content.size(),
                   content.c_str());
    }
    TEST_ASSERT(content == kNew);
  }

  // 长文件名（LFN 两条以上）/ Long name (multi-part LFN).
  const char* kLongName = "/this-is-a-very-long-file-name.txt";
  const char* kLong = "lfn-content\n";
  {
    LibXR::FatFile f(fat, kLongName, LibXR::FatFile::Mode::WRITE_NEW);
    TEST_ASSERT(f.IsValid());
    const LibXR::ConstRawData w = {kLong, std::strlen(kLong)};
    size_t len = std::strlen(kLong);
    TEST_ASSERT(f.Write(w, len) == LibXR::ErrorCode::OK);
    TEST_ASSERT(f.Sync() == LibXR::ErrorCode::OK);

    LibXR::FatFile r(fat, kLongName);
    TEST_ASSERT(r.IsValid());
    TEST_ASSERT(ReadAll(r) == kLong);
  }

  // 短名冲突：两个长名截断到同一基名，~n 必须生成不同短名。
  // FAT 大小写不敏感（同 Windows/Linux），大小写不构成冲突。
  // Short-name collision: two long names truncate to the same base, ~n
  // disambiguates. FAT lookup is case-insensitive, like Windows/Linux.
  {
    LibXR::FatFile a(fat, "/longnamea.txt", LibXR::FatFile::Mode::WRITE_NEW);
    TEST_ASSERT(a.IsValid());
    const LibXR::ConstRawData wa = {"aa", 2};
    size_t la = 2;
    TEST_ASSERT(a.Write(wa, la) == LibXR::ErrorCode::OK);
    TEST_ASSERT(a.Sync() == LibXR::ErrorCode::OK);

    LibXR::FatFile b(fat, "/longnameb.txt", LibXR::FatFile::Mode::WRITE_NEW);
    TEST_ASSERT(b.IsValid());
    const LibXR::ConstRawData wb = {"bb", 2};
    size_t lb = 2;
    TEST_ASSERT(b.Write(wb, lb) == LibXR::ErrorCode::OK);
    TEST_ASSERT(b.Sync() == LibXR::ErrorCode::OK);

    LibXR::FatFile ra(fat, "/longnamea.txt");
    LibXR::FatFile rb(fat, "/longnameb.txt");
    TEST_ASSERT(ra.IsValid() && rb.IsValid());
    TEST_ASSERT(ReadAll(ra) == "aa");
    TEST_ASSERT(ReadAll(rb) == "bb");
  }

  // 多簇写入（跨簇链）/ Multi-cluster write crossing the chain.
  {
    std::string big(4000, 'x');
    for (size_t i = 0; i < big.size(); ++i)
    {
      big[i] = static_cast<char>('A' + (i % 26));
    }
    LibXR::FatFile f(fat, "/big.bin", LibXR::FatFile::Mode::WRITE_NEW);
    TEST_ASSERT(f.IsValid());
    size_t len = big.size();
    TEST_ASSERT(f.Write({big.data(), big.size()}, len) == LibXR::ErrorCode::OK);
    TEST_ASSERT(len == big.size());
    TEST_ASSERT(f.Sync() == LibXR::ErrorCode::OK);

    LibXR::FatFile r(fat, "/big.bin");
    TEST_ASSERT(r.IsValid());
    TEST_ASSERT(ReadAll(r) == big);
  }

  // 追加与截断 / Append and truncate.
  {
    LibXR::FatFile f(fat, "/newfile.txt", LibXR::FatFile::Mode::WRITE_APPEND);
    TEST_ASSERT(f.IsValid());
    const LibXR::ConstRawData w = {"APPEND", 6};
    size_t len = 6;
    TEST_ASSERT(f.Write(w, len) == LibXR::ErrorCode::OK);
    TEST_ASSERT(f.Sync() == LibXR::ErrorCode::OK);
    TEST_ASSERT(f.Size() == std::strlen(kNew) + 6);

    LibXR::FatFile r(fat, "/newfile.txt");
    TEST_ASSERT(ReadAll(r) == std::string(kNew) + "APPEND");

    LibXR::FatFile t(fat, "/newfile.txt", LibXR::FatFile::Mode::READ_WRITE);
    TEST_ASSERT(t.IsValid());
    TEST_ASSERT(t.Truncate(5) == LibXR::ErrorCode::OK);
    LibXR::FatFile r2(fat, "/newfile.txt");
    TEST_ASSERT(r2.IsValid());
    TEST_ASSERT(r2.Size() == 5);
    TEST_ASSERT(ReadAll(r2) == "writt");
  }

  // 删除 + 最终 fsck / Delete then final fsck.
  {
    TEST_ASSERT(fat.Delete("/big.bin") == LibXR::ErrorCode::OK);
    TEST_ASSERT(!LibXR::FatFile(fat, "/big.bin").IsValid());
    const int rc = RunCommandRc(std::string("fsck.vfat -n ") + img);
    if (rc != 0)
    {
      RunCommand(std::string("fsck.vfat -n -v ") + img);
    }
    TEST_ASSERT(rc == 0);
  }
}

}  // namespace

void test_fat()
{
  // mtools 直接打开 /dev/tty 弹几何提示（PTY 下永远阻塞），用官方开关禁用。
  // mtools opens /dev/tty directly for geometry prompts (blocks forever under
  // a PTY); the official env switch disables the check.
  setenv("MTOOLS_SKIP_CHECK", "1", 1);

  // 测试临时目录（构建目录下，避开 /tmp 配额问题）/ Scratch dir under build.
  RunCommand(std::string("mkdir -p ") + FAT_TEST_DIR);

  if (!ToolExists("mkfs.vfat") || !ToolExists("mcopy") || !ToolExists("mmd"))
  {
    std::fprintf(stderr,
                 "FAT test: dosfstools (mkfs.vfat/mcopy/mmd) missing — SKIPPED. "
                 "Install dosfstools to run the M3a oracle tests.\n");
    return;
  }

  const char* kFat16Img = FAT_TEST_DIR "16.img";
  const char* kFat32Img = FAT_TEST_DIR "32.img";

  TestOneImage(kFat16Img, "16", false);
  TestOneImage(kFat32Img, "32", true);

  std::printf("FAT M3a tests passed (FAT16 + FAT32)\n");
}
