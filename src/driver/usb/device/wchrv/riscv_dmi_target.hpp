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
    const ErrorCode RESULT = sdi_.Transfer({addr, 0u, LibXR::Debug::Sdi::Op::NOP}, resp);
    data = resp.data;
    ack = resp.ack;
    return RESULT;
  }

  bool ReadDmStatus(uint32_t& dmstatus) { return DmiReadWord(DMI_DMSTATUS, dmstatus); }

  bool ReadWordByAbstract(uint32_t addr, uint32_t& data)
  {
    if (!DmiWriteWord(DMI_PROGBUF0, 0x0002A303u))  // lw x6, 0(x5)
    {
      return false;
    }
    if (!DmiWriteWord(DMI_PROGBUF1, 0x00100073u))  // ebreak
    {
      return false;
    }
    if (!DmiWriteWord(DMI_DATA0, addr))
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
    return DmiReadWord(DMI_DATA0, data);
  }

  bool WriteWordByAbstract(uint32_t addr, uint32_t data)
  {
    if (!DmiWriteWord(DMI_PROGBUF0, 0x0072A023u))  // sw x7, 0(x5)
    {
      return false;
    }
    if (!DmiWriteWord(DMI_PROGBUF1, 0x00100073u))  // ebreak
    {
      return false;
    }

    if (!DmiWriteWord(DMI_DATA0, addr))
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

    if (!DmiWriteWord(DMI_DATA0, data))
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
    if (!DmiWriteWord(DMI_PROGBUF0, 0x00728023u))  // sb x7, 0(x5)
    {
      return false;
    }
    if (!DmiWriteWord(DMI_PROGBUF1, 0x00100073u))  // ebreak
    {
      return false;
    }

    if (!DmiWriteWord(DMI_DATA0, addr))
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

    if (!DmiWriteWord(DMI_DATA0, static_cast<uint32_t>(data)))
    {
      return false;
    }
    if (!ClearAbstractCommandError())
    {
      return false;
    }
    return RunAbstractCommand(0x00271007u);  // x7 <- data0 with postexec
  }

  bool ReadWordWithTemporaryHalt(uint32_t addr, uint32_t& data)
  {
    bool need_resume = false;
    if (!EnsureHartHaltedForProbe(need_resume))
    {
      return false;
    }

    const bool READ_OK = ReadWordByAbstract(addr, data);
    TryResumeHartAfterProbe(need_resume);
    return READ_OK;
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
    const ErrorCode RESULT = sdi_.DmiReadTxn(addr, data, ack);
    return RESULT == ErrorCode::OK && ack == LibXR::Debug::Sdi::Ack::OK;
  }

  bool DmiWriteWord(uint8_t addr, uint32_t data)
  {
    LibXR::Debug::Sdi::Ack ack = LibXR::Debug::Sdi::Ack::PROTOCOL;
    const ErrorCode RESULT = sdi_.DmiWriteTxn(addr, data, ack);
    return RESULT == ErrorCode::OK && ack == LibXR::Debug::Sdi::Ack::OK;
  }

  bool ClearAbstractCommandError() { return DmiWriteWord(DMI_ABSTRACTCS, 0x00000700u); }

  bool WaitAbstractCommandDone()
  {
    for (uint8_t i = 0u; i < 32u; ++i)
    {
      uint32_t abstractcs = 0u;
      if (!DmiReadWord(DMI_ABSTRACTCS, abstractcs))
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
    if (!DmiWriteWord(DMI_COMMAND, command))
    {
      return false;
    }
    return WaitAbstractCommandDone();
  }

  bool EnsureHartHaltedForProbe(bool& need_resume)
  {
    need_resume = false;
    uint32_t dmstatus = 0u;
    if (!DmiReadWord(DMI_DMSTATUS, dmstatus))
    {
      return false;
    }
    if (IsDmHalted(dmstatus))
    {
      return true;
    }

    if (!DmiWriteWord(DMI_DMCONTROL, 0x80000001u))
    {
      return false;
    }

    for (uint8_t i = 0u; i < 32u; ++i)
    {
      if (!DmiReadWord(DMI_DMSTATUS, dmstatus))
      {
        return false;
      }
      if (IsDmHalted(dmstatus))
      {
        if (!DmiWriteWord(DMI_DMCONTROL, 0x00000001u))
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

    (void)DmiWriteWord(DMI_DMCONTROL, 0x40000001u);
    for (uint8_t i = 0u; i < 16u; ++i)
    {
      uint32_t dmstatus = 0u;
      if (!DmiReadWord(DMI_DMSTATUS, dmstatus))
      {
        break;
      }
      if (IsDmRunning(dmstatus))
      {
        break;
      }
    }
    (void)DmiWriteWord(DMI_DMCONTROL, 0x00000001u);
  }

 private:
  static constexpr uint8_t DMI_DATA0 = 0x04u;
  static constexpr uint8_t DMI_DMCONTROL = 0x10u;
  static constexpr uint8_t DMI_DMSTATUS = 0x11u;
  static constexpr uint8_t DMI_ABSTRACTCS = 0x16u;
  static constexpr uint8_t DMI_COMMAND = 0x17u;
  static constexpr uint8_t DMI_PROGBUF0 = 0x20u;
  static constexpr uint8_t DMI_PROGBUF1 = 0x21u;

  SdiPort& sdi_;
};

}  // namespace LibXR::USB
