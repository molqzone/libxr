#pragma once

#include <cstdint>

#include "debug/sdi.hpp"
#include "libxr_def.hpp"

namespace LibXR::USB
{
template <typename SdiPort>
class RiscvDmiTarget
{
 public:
  struct ProbeData
  {
    uint32_t chip_id = 0u;
    uint16_t flash_size_kb = 0u;
    uint32_t uid_word0 = 0u;
    uint32_t uid_word1 = 0u;
  };

  explicit RiscvDmiTarget(SdiPort& sdi_link) : sdi_(sdi_link) {}

  ErrorCode DmiRead(uint8_t addr, uint32_t& data, LibXR::Debug::Sdi::Ack& ack)
  {
    return sdi_.DmiReadTxn(addr, data, ack);
  }

  ErrorCode DmiWrite(uint8_t addr, uint32_t data, LibXR::Debug::Sdi::Ack& ack)
  {
    return sdi_.DmiWriteTxn(addr, data, ack);
  }

  ErrorCode DmiNop(uint8_t addr, uint32_t& data, LibXR::Debug::Sdi::Ack& ack)
  {
    LibXR::Debug::Sdi::Response resp = {};
    const ErrorCode ec = sdi_.Transfer({addr, 0u, LibXR::Debug::Sdi::Op::NOP}, resp);
    data = resp.data;
    ack = resp.ack;
    return ec;
  }

  bool ReadDmStatus(uint32_t& dmstatus) { return DmiReadWord(kDmiDmstatus, dmstatus); }

  bool ReadWordByAbstract(uint32_t addr, uint32_t& data)
  {
    if (!DmiWriteWord(kDmiProgbuf0, 0x0002A303u))  // lw x6, 0(x5)
    {
      return false;
    }
    if (!DmiWriteWord(kDmiProgbuf1, 0x00100073u))  // ebreak
    {
      return false;
    }
    if (!DmiWriteWord(kDmiData0, addr))
    {
      return false;
    }
    if (!ClearAbstractCommandError())
    {
      return false;
    }
    if (!RunAbstractCommand(0x00271005u))  // x5 <- data0 with postexec
    {
      return false;
    }
    if (!RunAbstractCommand(0x00221006u))  // data0 <- x6
    {
      return false;
    }
    return DmiReadWord(kDmiData0, data);
  }

  bool WriteWordByAbstract(uint32_t addr, uint32_t data)
  {
    if (!DmiWriteWord(kDmiProgbuf0, 0x0072A023u))  // sw x7, 0(x5)
    {
      return false;
    }
    if (!DmiWriteWord(kDmiProgbuf1, 0x00100073u))  // ebreak
    {
      return false;
    }

    if (!DmiWriteWord(kDmiData0, addr))
    {
      return false;
    }
    if (!ClearAbstractCommandError())
    {
      return false;
    }
    if (!RunAbstractCommand(0x00231005u))  // x5 <- data0
    {
      return false;
    }

    if (!DmiWriteWord(kDmiData0, data))
    {
      return false;
    }
    if (!ClearAbstractCommandError())
    {
      return false;
    }
    return RunAbstractCommand(0x00271007u);  // x7 <- data0 with postexec
  }

  bool WriteByteByAbstract(uint32_t addr, uint8_t data)
  {
    if (!DmiWriteWord(kDmiProgbuf0, 0x00728023u))  // sb x7, 0(x5)
    {
      return false;
    }
    if (!DmiWriteWord(kDmiProgbuf1, 0x00100073u))  // ebreak
    {
      return false;
    }

    if (!DmiWriteWord(kDmiData0, addr))
    {
      return false;
    }
    if (!ClearAbstractCommandError())
    {
      return false;
    }
    if (!RunAbstractCommand(0x00231005u))  // x5 <- data0
    {
      return false;
    }

    if (!DmiWriteWord(kDmiData0, static_cast<uint32_t>(data)))
    {
      return false;
    }
    if (!ClearAbstractCommandError())
    {
      return false;
    }
    return RunAbstractCommand(0x00271007u);  // x7 <- data0 with postexec
  }

  bool ProbeChipIdentity(ProbeData& out)
  {
    bool need_resume = false;
    if (!EnsureHartHaltedForProbe(need_resume))
    {
      return false;
    }

    bool ok = true;
    uint32_t chip_id = 0u;
    do
    {
      if (!ReadWordByAbstract(kChipIdAddress, chip_id))
      {
        ok = false;
        break;
      }
      if (chip_id == 0u || chip_id == 0xFFFFFFFFu)
      {
        ok = false;
        break;
      }

      out.chip_id = chip_id;

      uint32_t flash_size = 0u;
      if (ReadWordByAbstract(kFlashSizeAddress, flash_size))
      {
        out.flash_size_kb = static_cast<uint16_t>(flash_size & 0xFFFFu);
      }

      uint32_t uid0 = 0u;
      if (ReadWordByAbstract(kUidWord0Address, uid0))
      {
        out.uid_word0 = uid0;
      }

      uint32_t uid1 = 0u;
      if (ReadWordByAbstract(kUidWord1Address, uid1))
      {
        out.uid_word1 = uid1;
      }
    } while (false);

    TryResumeHartAfterProbe(need_resume);
    return ok;
  }

  bool BeginHartHaltSession(bool& need_resume)
  {
    return EnsureHartHaltedForProbe(need_resume);
  }

  void EndHartHaltSession(bool need_resume)
  {
    TryResumeHartAfterProbe(need_resume);
  }

 private:
  static bool IsDmHalted(uint32_t dmstatus)
  {
    return (dmstatus & (1u << 9u)) != 0u && (dmstatus & (1u << 8u)) != 0u;
  }

  static bool IsDmRunning(uint32_t dmstatus)
  {
    return (dmstatus & (1u << 11u)) != 0u && (dmstatus & (1u << 10u)) != 0u;
  }

  bool DmiReadWord(uint8_t addr, uint32_t& data)
  {
    LibXR::Debug::Sdi::Ack ack = LibXR::Debug::Sdi::Ack::PROTOCOL;
    const ErrorCode ec = sdi_.DmiReadTxn(addr, data, ack);
    return ec == ErrorCode::OK && ack == LibXR::Debug::Sdi::Ack::OK;
  }

  bool DmiWriteWord(uint8_t addr, uint32_t data)
  {
    LibXR::Debug::Sdi::Ack ack = LibXR::Debug::Sdi::Ack::PROTOCOL;
    const ErrorCode ec = sdi_.DmiWriteTxn(addr, data, ack);
    return ec == ErrorCode::OK && ack == LibXR::Debug::Sdi::Ack::OK;
  }

  bool ClearAbstractCommandError() { return DmiWriteWord(kDmiAbstractcs, 0x00000700u); }

  bool WaitAbstractCommandDone()
  {
    for (uint8_t i = 0u; i < 32u; ++i)
    {
      uint32_t abstractcs = 0u;
      if (!DmiReadWord(kDmiAbstractcs, abstractcs))
      {
        return false;
      }
      if ((abstractcs & (1u << 12u)) != 0u)
      {
        continue;
      }
      if (((abstractcs >> 8u) & 0x7u) != 0u)
      {
        (void)ClearAbstractCommandError();
        return false;
      }
      return true;
    }
    return false;
  }

  bool RunAbstractCommand(uint32_t command)
  {
    if (!DmiWriteWord(kDmiCommand, command))
    {
      return false;
    }
    return WaitAbstractCommandDone();
  }

  bool EnsureHartHaltedForProbe(bool& need_resume)
  {
    need_resume = false;
    uint32_t dmstatus = 0u;
    if (!DmiReadWord(kDmiDmstatus, dmstatus))
    {
      return false;
    }
    if (IsDmHalted(dmstatus))
    {
      return true;
    }

    if (!DmiWriteWord(kDmiDmcontrol, 0x80000001u))
    {
      return false;
    }

    for (uint8_t i = 0u; i < 32u; ++i)
    {
      if (!DmiReadWord(kDmiDmstatus, dmstatus))
      {
        return false;
      }
      if (IsDmHalted(dmstatus))
      {
        if (!DmiWriteWord(kDmiDmcontrol, 0x00000001u))
        {
          return false;
        }
        need_resume = true;
        return true;
      }
    }
    return false;
  }

  void TryResumeHartAfterProbe(bool need_resume)
  {
    if (!need_resume)
    {
      return;
    }

    (void)DmiWriteWord(kDmiDmcontrol, 0x40000001u);
    for (uint8_t i = 0u; i < 16u; ++i)
    {
      uint32_t dmstatus = 0u;
      if (!DmiReadWord(kDmiDmstatus, dmstatus))
      {
        break;
      }
      if (IsDmRunning(dmstatus))
      {
        break;
      }
    }
    (void)DmiWriteWord(kDmiDmcontrol, 0x00000001u);
  }

 private:
  static constexpr uint8_t kDmiData0 = 0x04u;
  static constexpr uint8_t kDmiDmcontrol = 0x10u;
  static constexpr uint8_t kDmiDmstatus = 0x11u;
  static constexpr uint8_t kDmiAbstractcs = 0x16u;
  static constexpr uint8_t kDmiCommand = 0x17u;
  static constexpr uint8_t kDmiProgbuf0 = 0x20u;
  static constexpr uint8_t kDmiProgbuf1 = 0x21u;
  static constexpr uint32_t kChipIdAddress = 0x1FFFF704u;
  static constexpr uint32_t kFlashSizeAddress = 0x1FFFF7E0u;
  static constexpr uint32_t kUidWord0Address = 0x1FFFF7E8u;
  static constexpr uint32_t kUidWord1Address = 0x1FFFF7ECu;

  SdiPort& sdi_;
};

}  // namespace LibXR::USB
