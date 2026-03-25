#pragma once

#include <array>
#include <cstdint>

#include "sdi.hpp"
#include "swd_protocol.hpp"

namespace LibXR::Debug
{
/**
 * @brief WCH SDI transport implemented over the RVSWD-style two-wire link.
 *
 * BitPort contract:
 * - ErrorCode SetClockHz(uint32_t)
 * - void Close()
 * - ErrorCode LineReset()
 * - ErrorCode WakeRvSwd()
 * - void IdleClocks(uint32_t)
 * - ErrorCode SeqWriteBits(uint32_t, const uint8_t*)
 * - ErrorCode SeqReadBits(uint32_t, uint8_t*)
 */
template <typename BitPort>
class SdiOverSwd final : public Sdi
{
 public:
  struct TransferDebugSnapshot
  {
    uint8_t stage = 0u;
    int8_t last_ec = 0;
    uint8_t last_req_addr = 0u;
    uint8_t last_req_tail = 0u;
    uint8_t last_resp_addr = 0u;
    uint8_t last_resp_tail = 0u;
    uint8_t last_parity_rx = 0u;
    uint8_t last_parity_calc = 0u;
    uint32_t last_req_data = 0u;
    uint32_t last_resp_data = 0u;
    uint32_t last_tx_lo = 0u;
    uint32_t last_tx_hi = 0u;
    uint32_t last_rx_lo = 0u;
    uint32_t last_rx_hi = 0u;
  };

  struct RawReadCapture
  {
    uint8_t dmi_addr = 0u;
    uint8_t bit_count = 0u;
    int8_t last_ec = static_cast<int8_t>(ErrorCode::OK);
    std::array<uint8_t, 12u> raw_bits = {};
  };

  struct SwdTransferCapture
  {
    uint8_t flags = 0u;
    uint8_t addr2b = 0u;
    int8_t last_ec = static_cast<int8_t>(ErrorCode::OK);
    uint8_t ack = 0x07u;
    uint8_t parity_ok = 0u;
    uint32_t wdata = 0u;
    uint32_t rdata = 0u;
  };

  struct SwdRawCapture
  {
    uint8_t flags = 0u;
    uint8_t addr2b = 0u;
    uint8_t bit_count = 0u;
    int8_t last_ec = static_cast<int8_t>(ErrorCode::OK);
    std::array<uint8_t, 12u> raw_bits = {};
  };

  struct SwdBatchOp
  {
    uint8_t flags = 0u;
    uint8_t addr2b = 0u;
    uint32_t wdata = 0u;
  };

  struct SwdBatchResult
  {
    uint8_t flags = 0u;
    uint8_t addr2b = 0u;
    int8_t last_ec = static_cast<int8_t>(ErrorCode::OK);
    uint8_t ack = 0x07u;
    uint8_t parity_ok = 0u;
    uint32_t rdata = 0u;
  };

  struct SwdBatchCapture
  {
    uint8_t batch_flags = 0u;
    uint8_t count = 0u;
    int8_t last_ec = static_cast<int8_t>(ErrorCode::OK);
    std::array<SwdBatchResult, 8u> results = {};
  };

  static inline volatile TransferDebugSnapshot debug_snapshot_ = {};

  explicit SdiOverSwd(BitPort& bit_port) : port_(bit_port) {}
  ~SdiOverSwd() override = default;

  SdiOverSwd(const SdiOverSwd&) = delete;
  SdiOverSwd& operator=(const SdiOverSwd&) = delete;

  ErrorCode SetClockHz(uint32_t hz) override { return port_.SetClockHz(hz); }

  void Close() override
  {
    entered_sdi_ = false;
    port_.Close();
  }

  ErrorCode LineReset() override
  {
    entered_sdi_ = false;
    return port_.LineReset();
  }

  ErrorCode EnterSdi() override
  {
    entered_sdi_ = false;
    debug_snapshot_.stage = 1u;
    debug_snapshot_.last_ec = static_cast<int8_t>(ErrorCode::OK);

    const ErrorCode RESULT = port_.WakeRvSwd();
    if (RESULT != ErrorCode::OK)
    {
      debug_snapshot_.last_ec = static_cast<int8_t>(RESULT);
      return RESULT;
    }

    port_.IdleClocks(2u);
    entered_sdi_ = true;
    debug_snapshot_.stage = 2u;
    return ErrorCode::OK;
  }

  ErrorCode Transfer(const Request& req, Response& resp) override
  {
    resp.addr = req.addr;
    resp.data = 0u;
    resp.ack = Ack::PROTOCOL;

    if (!entered_sdi_)
    {
      const ErrorCode RESULT = EnterSdi();
      if (RESULT != ErrorCode::OK)
      {
        return RESULT;
      }
    }

    if (req.op == Op::NOP)
    {
      debug_snapshot_.stage = 0x0Fu;
      debug_snapshot_.last_req_addr = 0u;
      debug_snapshot_.last_req_tail = 0u;
      debug_snapshot_.last_req_data = 0u;
      debug_snapshot_.last_resp_addr = 0u;
      debug_snapshot_.last_resp_tail = 0u;
      debug_snapshot_.last_resp_data = 0u;
      debug_snapshot_.last_tx_lo = 0u;
      debug_snapshot_.last_tx_hi = 0u;
      debug_snapshot_.last_rx_lo = 0u;
      debug_snapshot_.last_rx_hi = 0u;
      debug_snapshot_.last_ec = static_cast<int8_t>(ErrorCode::OK);
      resp.ack = Ack::OK;
      return ErrorCode::OK;
    }

    ErrorCode result = ErrorCode::ARG_ERR;
    if (req.op == Op::READ)
    {
      uint32_t data = 0u;
      result = ReadWordRaw(req.addr, data);
      if (result == ErrorCode::OK)
      {
        resp.data = data;
      }
    }
    else if (req.op == Op::WRITE)
    {
      result = WriteWordRaw(req.addr, req.data);
      if (result == ErrorCode::OK)
      {
        resp.data = req.data;
      }
    }

    if (result != ErrorCode::OK)
    {
      entered_sdi_ = false;
      debug_snapshot_.last_ec = static_cast<int8_t>(result);
      return result;
    }

    resp.ack = Ack::OK;
    return ErrorCode::OK;
  }

  void IdleClocks(uint32_t cycles) override { port_.IdleClocks(cycles); }

  ErrorCode DebugCaptureReadBits(uint8_t dmi_addr, uint8_t bit_count, RawReadCapture& capture)
  {
    capture = {};
    capture.dmi_addr = dmi_addr;
    capture.bit_count = bit_count;

    if (bit_count == 0u || bit_count > static_cast<uint8_t>(capture.raw_bits.size() * 8u))
    {
      capture.last_ec = static_cast<int8_t>(ErrorCode::ARG_ERR);
      return ErrorCode::ARG_ERR;
    }

    if (!entered_sdi_)
    {
      const ErrorCode ENTER_RESULT = EnterSdi();
      capture.last_ec = static_cast<int8_t>(ENTER_RESULT);
      if (ENTER_RESULT != ErrorCode::OK)
      {
        return ENTER_RESULT;
      }
    }

    const uint16_t HEADER = BuildReadPrefix(dmi_addr);

    debug_snapshot_.stage = 0x30u;
    debug_snapshot_.last_req_addr = dmi_addr;
    debug_snapshot_.last_req_tail = 0x81u;
    debug_snapshot_.last_resp_addr = dmi_addr;
    debug_snapshot_.last_resp_tail = bit_count;
    debug_snapshot_.last_resp_data = 0u;
    debug_snapshot_.last_tx_lo = HEADER;
    debug_snapshot_.last_tx_hi = 0u;
    debug_snapshot_.last_rx_lo = 0u;
    debug_snapshot_.last_rx_hi = 0u;
    debug_snapshot_.last_ec = static_cast<int8_t>(ErrorCode::OK);

    ErrorCode result = port_.BeginRvSwdFrame();
    capture.last_ec = static_cast<int8_t>(result);
    if (result != ErrorCode::OK)
    {
      debug_snapshot_.last_ec = capture.last_ec;
      entered_sdi_ = false;
      return result;
    }

    result = EncodeAndWriteMsbFirst(port_, HEADER, READ_PREFIX_BITS);
    capture.last_ec = static_cast<int8_t>(result);
    if (result != ErrorCode::OK)
    {
      debug_snapshot_.last_ec = capture.last_ec;
      entered_sdi_ = false;
      return result;
    }

    debug_snapshot_.stage = 0x31u;
    result = port_.SeqReadBits(bit_count, capture.raw_bits.data());
    capture.last_ec = static_cast<int8_t>(result);
    if (result != ErrorCode::OK)
    {
      debug_snapshot_.last_ec = capture.last_ec;
      entered_sdi_ = false;
      return result;
    }

    uint64_t preview = 0u;
    const uint32_t PREVIEW_BITS = (bit_count < 64u) ? bit_count : 64u;
    for (uint32_t i = 0u; i < PREVIEW_BITS; ++i)
    {
      if (GetBitLsb(capture.raw_bits.data(), i))
      {
        preview |= (static_cast<uint64_t>(1u) << i);
      }
    }
    debug_snapshot_.last_rx_lo = static_cast<uint32_t>(preview & 0xFFFF'FFFFu);
    debug_snapshot_.last_rx_hi = static_cast<uint32_t>(preview >> 32u);

    debug_snapshot_.stage = 0x32u;
    result = port_.EndRvSwdFrame();
    capture.last_ec = static_cast<int8_t>(result);
    debug_snapshot_.last_ec = capture.last_ec;
    if (result != ErrorCode::OK)
    {
      entered_sdi_ = false;
      return result;
    }

    debug_snapshot_.stage = 0x33u;
    return ErrorCode::OK;
  }

  ErrorCode DebugSwdTransfer(uint8_t flags, uint8_t addr2b, uint32_t wdata,
                             SwdTransferCapture& capture)
  {
    capture = {};
    capture.flags = flags;
    capture.addr2b = static_cast<uint8_t>(addr2b & 0x03u);
    capture.wdata = wdata;

    const bool FORCE_REENTER = (flags & 0x40u) != 0u;
    const bool WRAP_RVSWD_FRAME = (flags & 0x80u) != 0u;

    if (FORCE_REENTER)
    {
      entered_sdi_ = false;
    }

    if (!entered_sdi_)
    {
      const ErrorCode ENTER_RESULT = EnterSdi();
      capture.last_ec = static_cast<int8_t>(ENTER_RESULT);
      if (ENTER_RESULT != ErrorCode::OK)
      {
        return ENTER_RESULT;
      }
    }

    debug_snapshot_.stage = 0x40u;
    debug_snapshot_.last_req_addr = capture.addr2b;
    debug_snapshot_.last_req_tail = flags;
    debug_snapshot_.last_req_data = wdata;
    debug_snapshot_.last_resp_addr = capture.addr2b;
    debug_snapshot_.last_resp_tail = 0u;
    debug_snapshot_.last_resp_data = 0u;
    debug_snapshot_.last_parity_rx = 0u;
    debug_snapshot_.last_parity_calc = 0u;
    debug_snapshot_.last_tx_lo = wdata;
    debug_snapshot_.last_tx_hi = 0u;
    debug_snapshot_.last_rx_lo = 0u;
    debug_snapshot_.last_rx_hi = 0u;
    debug_snapshot_.last_ec = static_cast<int8_t>(ErrorCode::OK);

    LibXR::Debug::SwdProtocol::Request request = {};
    request.port = ((flags & 0x01u) != 0u) ? LibXR::Debug::SwdProtocol::Port::AP
                                           : LibXR::Debug::SwdProtocol::Port::DP;
    request.rnw = (flags & 0x02u) != 0u;
    request.addr2b = capture.addr2b;
    request.wdata = wdata;

    LibXR::Debug::SwdProtocol::Response response = {};
    ErrorCode result = ErrorCode::OK;
    const bool USE_TXN_SEMANTICS = (flags & 0x10u) != 0u;
    const bool USE_RETRY_ONLY = (flags & 0x20u) != 0u;
    if (WRAP_RVSWD_FRAME)
    {
      result = port_.BeginRvSwdFrame();
      capture.last_ec = static_cast<int8_t>(result);
      debug_snapshot_.last_ec = capture.last_ec;
      if (result != ErrorCode::OK)
      {
        entered_sdi_ = false;
        return result;
      }
    }

    if (USE_TXN_SEMANTICS && request.port == LibXR::Debug::SwdProtocol::Port::AP && request.rnw)
    {
      LibXR::Debug::SwdProtocol::Ack ack = LibXR::Debug::SwdProtocol::Ack::PROTOCOL;
      uint32_t rdata = 0u;
      result = port_.ApReadTxn(capture.addr2b, rdata, ack);
      capture.last_ec = static_cast<int8_t>(result);
      capture.ack = static_cast<uint8_t>(ack);
      capture.parity_ok =
          (result == ErrorCode::OK && ack == LibXR::Debug::SwdProtocol::Ack::OK) ? 1u : 0u;
      capture.rdata = rdata;
    }
    else
    {
      result = (USE_TXN_SEMANTICS || USE_RETRY_ONLY) ? port_.TransferWithRetry(request, response)
                                                     : port_.Transfer(request, response);
      capture.last_ec = static_cast<int8_t>(result);
      capture.ack = static_cast<uint8_t>(response.ack);
      capture.parity_ok = response.parity_ok ? 1u : 0u;
      capture.rdata = response.rdata;
    }

    if (WRAP_RVSWD_FRAME)
    {
      const ErrorCode END_RESULT = port_.EndRvSwdFrame();
      if (result == ErrorCode::OK)
      {
        result = END_RESULT;
        capture.last_ec = static_cast<int8_t>(result);
      }
    }

    debug_snapshot_.last_resp_tail = capture.ack;
    debug_snapshot_.last_resp_data = capture.rdata;
    debug_snapshot_.last_parity_rx = capture.parity_ok;
    debug_snapshot_.last_rx_lo = capture.rdata;
    debug_snapshot_.stage = 0x41u;
    debug_snapshot_.last_ec = capture.last_ec;

    if (result != ErrorCode::OK)
    {
      entered_sdi_ = false;
      return result;
    }

    return ErrorCode::OK;
  }

  ErrorCode DebugCaptureSwdBits(uint8_t flags, uint8_t addr2b, uint8_t bit_count,
                                SwdRawCapture& capture)
  {
    capture = {};
    capture.flags = flags;
    capture.addr2b = static_cast<uint8_t>(addr2b & 0x03u);
    capture.bit_count = bit_count;

    if (bit_count == 0u || bit_count > static_cast<uint8_t>(capture.raw_bits.size() * 8u))
    {
      capture.last_ec = static_cast<int8_t>(ErrorCode::ARG_ERR);
      return ErrorCode::ARG_ERR;
    }

    const bool FORCE_REENTER = (flags & 0x40u) != 0u;
    const bool WRAP_RVSWD_FRAME = (flags & 0x80u) != 0u;

    if (FORCE_REENTER)
    {
      entered_sdi_ = false;
    }

    if (!entered_sdi_)
    {
      const ErrorCode ENTER_RESULT = EnterSdi();
      capture.last_ec = static_cast<int8_t>(ENTER_RESULT);
      if (ENTER_RESULT != ErrorCode::OK)
      {
        return ENTER_RESULT;
      }
    }

    const uint8_t REQUEST_BYTE = EncodeSwdRequestByte((flags & 0x01u) != 0u, (flags & 0x02u) != 0u,
                                                      capture.addr2b);
    debug_snapshot_.stage = 0x50u;
    debug_snapshot_.last_req_addr = capture.addr2b;
    debug_snapshot_.last_req_tail = flags;
    debug_snapshot_.last_req_data = REQUEST_BYTE;
    debug_snapshot_.last_resp_addr = capture.addr2b;
    debug_snapshot_.last_resp_tail = bit_count;
    debug_snapshot_.last_resp_data = 0u;
    debug_snapshot_.last_tx_lo = REQUEST_BYTE;
    debug_snapshot_.last_tx_hi = 0u;
    debug_snapshot_.last_rx_lo = 0u;
    debug_snapshot_.last_rx_hi = 0u;
    debug_snapshot_.last_ec = static_cast<int8_t>(ErrorCode::OK);

    ErrorCode result = ErrorCode::OK;
    if (WRAP_RVSWD_FRAME)
    {
      result = port_.BeginRvSwdFrame();
      capture.last_ec = static_cast<int8_t>(result);
      if (result != ErrorCode::OK)
      {
        debug_snapshot_.last_ec = capture.last_ec;
        entered_sdi_ = false;
        return result;
      }
    }

    result = port_.SeqWriteBits(8u, &REQUEST_BYTE);
    capture.last_ec = static_cast<int8_t>(result);
    if (result == ErrorCode::OK)
    {
      debug_snapshot_.stage = 0x51u;
      result = port_.SeqReadBits(bit_count, capture.raw_bits.data());
      capture.last_ec = static_cast<int8_t>(result);
    }

    if (WRAP_RVSWD_FRAME)
    {
      const ErrorCode END_RESULT = port_.EndRvSwdFrame();
      if (result == ErrorCode::OK)
      {
        result = END_RESULT;
        capture.last_ec = static_cast<int8_t>(result);
      }
    }

    uint64_t preview = 0u;
    const uint32_t PREVIEW_BITS = (bit_count < 64u) ? bit_count : 64u;
    for (uint32_t i = 0u; i < PREVIEW_BITS; ++i)
    {
      if (GetBitLsb(capture.raw_bits.data(), i))
      {
        preview |= (static_cast<uint64_t>(1u) << i);
      }
    }
    debug_snapshot_.last_rx_lo = static_cast<uint32_t>(preview & 0xFFFF'FFFFu);
    debug_snapshot_.last_rx_hi = static_cast<uint32_t>(preview >> 32u);
    debug_snapshot_.stage = 0x52u;
    debug_snapshot_.last_ec = capture.last_ec;

    if (result != ErrorCode::OK)
    {
      entered_sdi_ = false;
      return result;
    }

    return ErrorCode::OK;
  }

  ErrorCode DebugSwdBatch(uint8_t batch_flags, const SwdBatchOp* ops, uint8_t count,
                          SwdBatchCapture& capture)
  {
    capture = {};
    capture.batch_flags = batch_flags;
    capture.count = count;

    if (!ops || count == 0u || count > static_cast<uint8_t>(capture.results.size()))
    {
      capture.last_ec = static_cast<int8_t>(ErrorCode::ARG_ERR);
      return ErrorCode::ARG_ERR;
    }

    if ((batch_flags & 0x01u) != 0u)
    {
      entered_sdi_ = false;
    }

    if (!entered_sdi_)
    {
      const ErrorCode ENTER_RESULT = EnterSdi();
      capture.last_ec = static_cast<int8_t>(ENTER_RESULT);
      if (ENTER_RESULT != ErrorCode::OK)
      {
        return ENTER_RESULT;
      }
    }

    debug_snapshot_.stage = 0x60u;
    debug_snapshot_.last_ec = static_cast<int8_t>(ErrorCode::OK);
    debug_snapshot_.last_req_addr = count;
    debug_snapshot_.last_req_tail = batch_flags;
    debug_snapshot_.last_req_data = 0u;
    debug_snapshot_.last_resp_addr = 0u;
    debug_snapshot_.last_resp_tail = 0u;
    debug_snapshot_.last_resp_data = 0u;
    debug_snapshot_.last_parity_rx = 0u;
    debug_snapshot_.last_parity_calc = 0u;
    debug_snapshot_.last_tx_lo = 0u;
    debug_snapshot_.last_tx_hi = 0u;
    debug_snapshot_.last_rx_lo = 0u;
    debug_snapshot_.last_rx_hi = 0u;

    const bool WRAP_PER_OP = (batch_flags & 0x80u) != 0u;
    ErrorCode result = ErrorCode::OK;
    if (!WRAP_PER_OP)
    {
      result = port_.BeginRvSwdFrame();
      capture.last_ec = static_cast<int8_t>(result);
      if (result != ErrorCode::OK)
      {
        debug_snapshot_.last_ec = capture.last_ec;
        entered_sdi_ = false;
        return result;
      }
    }

    for (uint8_t i = 0u; i < count; ++i)
    {
      auto& out = capture.results[i];
      out.flags = ops[i].flags;
      out.addr2b = static_cast<uint8_t>(ops[i].addr2b & 0x03u);

      LibXR::Debug::SwdProtocol::Request request = {};
      request.port = ((ops[i].flags & 0x01u) != 0u) ? LibXR::Debug::SwdProtocol::Port::AP
                                                    : LibXR::Debug::SwdProtocol::Port::DP;
      request.rnw = (ops[i].flags & 0x02u) != 0u;
      request.addr2b = out.addr2b;
      request.wdata = ops[i].wdata;

      LibXR::Debug::SwdProtocol::Response response = {};
      debug_snapshot_.stage = static_cast<uint8_t>(0x61u + i);
      debug_snapshot_.last_req_addr = out.addr2b;
      debug_snapshot_.last_req_tail = ops[i].flags;
      debug_snapshot_.last_req_data = ops[i].wdata;

      if (WRAP_PER_OP)
      {
        result = port_.BeginRvSwdFrame();
        out.last_ec = static_cast<int8_t>(result);
        capture.last_ec = out.last_ec;
        if (result != ErrorCode::OK)
        {
          break;
        }
      }

      const bool USE_TXN_SEMANTICS = (ops[i].flags & 0x10u) != 0u;
      const bool USE_RETRY_ONLY = (ops[i].flags & 0x20u) != 0u;
      if (USE_TXN_SEMANTICS && request.port == LibXR::Debug::SwdProtocol::Port::AP &&
          request.rnw)
      {
        LibXR::Debug::SwdProtocol::Ack ack = LibXR::Debug::SwdProtocol::Ack::PROTOCOL;
        uint32_t rdata = 0u;
        result = port_.ApReadTxn(out.addr2b, rdata, ack);
        out.last_ec = static_cast<int8_t>(result);
        out.ack = static_cast<uint8_t>(ack);
        out.parity_ok =
            (result == ErrorCode::OK && ack == LibXR::Debug::SwdProtocol::Ack::OK) ? 1u : 0u;
        out.rdata = rdata;
      }
      else
      {
        result = (USE_TXN_SEMANTICS || USE_RETRY_ONLY) ? port_.TransferWithRetry(request, response)
                                                       : port_.Transfer(request, response);
        out.last_ec = static_cast<int8_t>(result);
        out.ack = static_cast<uint8_t>(response.ack);
        out.parity_ok = response.parity_ok ? 1u : 0u;
        out.rdata = response.rdata;
      }

      if (WRAP_PER_OP)
      {
        const ErrorCode END_RESULT = port_.EndRvSwdFrame();
        if (result == ErrorCode::OK)
        {
          result = END_RESULT;
          out.last_ec = static_cast<int8_t>(result);
        }
      }

      capture.last_ec = out.last_ec;

      debug_snapshot_.last_resp_tail = out.ack;
      debug_snapshot_.last_resp_data = out.rdata;
      debug_snapshot_.last_parity_rx = out.parity_ok;
      debug_snapshot_.last_rx_lo = out.rdata;
      debug_snapshot_.last_ec = out.last_ec;

      if (result != ErrorCode::OK)
      {
        break;
      }
    }

    if (!WRAP_PER_OP)
    {
      const ErrorCode END_RESULT = port_.EndRvSwdFrame();
      if (result == ErrorCode::OK)
      {
        result = END_RESULT;
        capture.last_ec = static_cast<int8_t>(result);
      }
    }

    if (result != ErrorCode::OK)
    {
      capture.last_ec = static_cast<int8_t>(result);
    }

    debug_snapshot_.stage = 0x6Fu;
    debug_snapshot_.last_ec = capture.last_ec;
    if (result != ErrorCode::OK)
    {
      entered_sdi_ = false;
      return result;
    }

    return ErrorCode::OK;
  }

 private:
  static constexpr uint32_t RVSWD_ADDR_BITS = 7u;
  static constexpr uint32_t RVSWD_OP_BITS = 1u;
  static constexpr uint32_t RVSWD_HEADER_PARITY_BITS = 1u;
  static constexpr uint32_t RVSWD_HOST_PAD_BITS = 5u;
  static constexpr uint32_t DATA_BITS = 32u;
  static constexpr uint32_t RVSWD_DATA_PARITY_BITS = 1u;
  static constexpr uint32_t RVSWD_HOST_TAIL_BITS = 5u;
  static constexpr uint32_t READ_PREFIX_BITS =
      RVSWD_ADDR_BITS + RVSWD_OP_BITS + RVSWD_HEADER_PARITY_BITS + RVSWD_HOST_PAD_BITS;
  static constexpr uint32_t WRITE_FRAME_BITS =
      READ_PREFIX_BITS + DATA_BITS + RVSWD_DATA_PARITY_BITS + RVSWD_HOST_TAIL_BITS;
  static constexpr uint8_t RVSWD_PAD_PREFIX = 0x15u;  // 10101
  static constexpr uint8_t RVSWD_TAIL = 0x17u;        // 10111
  static constexpr uint8_t DMI_ADDR_MASK = 0x7Fu;

  using WireBuffer = std::array<uint8_t, 8u>;

  static uint8_t Parity7PlusOp(uint8_t dmi_addr, bool write)
  {
    uint8_t parity = 0u;
    const uint8_t ADDR = static_cast<uint8_t>(dmi_addr & DMI_ADDR_MASK);
    for (uint8_t bit = 0u; bit < RVSWD_ADDR_BITS; ++bit)
    {
      parity ^= static_cast<uint8_t>((ADDR >> bit) & 0x01u);
    }
    if (write)
    {
      parity ^= 0x01u;
    }
    return static_cast<uint8_t>(parity & 0x01u);
  }

  static uint8_t Parity32(uint32_t value)
  {
    value ^= value >> 16u;
    value ^= value >> 8u;
    value ^= value >> 4u;
    value &= 0x0Fu;
    static constexpr uint8_t LUT[16] = {0, 1, 1, 0, 1, 0, 0, 1,
                                        1, 0, 0, 1, 0, 1, 1, 0};
    return LUT[value];
  }

  static uint16_t BuildReadPrefix(uint8_t dmi_addr)
  {
    const uint16_t ADDR = static_cast<uint16_t>(dmi_addr & DMI_ADDR_MASK);
    const uint16_t PARITY = static_cast<uint16_t>(Parity7PlusOp(dmi_addr, false));
    return static_cast<uint16_t>((ADDR << (RVSWD_OP_BITS + RVSWD_HEADER_PARITY_BITS +
                                           RVSWD_HOST_PAD_BITS)) |
                                 (PARITY << RVSWD_HOST_PAD_BITS) | RVSWD_PAD_PREFIX);
  }

  static uint64_t BuildWriteFrame(uint8_t dmi_addr, uint32_t data)
  {
    const uint64_t ADDR = static_cast<uint64_t>(dmi_addr & DMI_ADDR_MASK);
    const uint64_t PREFIX_PARITY = static_cast<uint64_t>(Parity7PlusOp(dmi_addr, true));
    const uint64_t PREFIX =
        (ADDR << (RVSWD_OP_BITS + RVSWD_HEADER_PARITY_BITS + RVSWD_HOST_PAD_BITS)) |
        (static_cast<uint64_t>(1u) << (RVSWD_HEADER_PARITY_BITS + RVSWD_HOST_PAD_BITS)) |
        (PREFIX_PARITY << RVSWD_HOST_PAD_BITS) | RVSWD_PAD_PREFIX;
    const uint64_t DATA_PARITY = static_cast<uint64_t>(Parity32(data));
    return (PREFIX << (DATA_BITS + RVSWD_DATA_PARITY_BITS + RVSWD_HOST_TAIL_BITS)) |
           (static_cast<uint64_t>(data) << (RVSWD_DATA_PARITY_BITS + RVSWD_HOST_TAIL_BITS)) |
           (DATA_PARITY << RVSWD_HOST_TAIL_BITS) | RVSWD_TAIL;
  }

  static uint8_t EncodeSwdRequestByte(bool apndp, bool rnw, uint8_t addr2b)
  {
    const uint8_t A2 = static_cast<uint8_t>(addr2b & 0x01u);
    const uint8_t A3 = static_cast<uint8_t>((addr2b >> 1u) & 0x01u);
    const uint8_t PARITY = static_cast<uint8_t>((static_cast<uint8_t>(apndp) ^
                                                 static_cast<uint8_t>(rnw) ^ A2 ^ A3) &
                                                0x01u);
    return static_cast<uint8_t>((1u << 0u) | (static_cast<uint8_t>(apndp) << 1u) |
                                (static_cast<uint8_t>(rnw) << 2u) | (A2 << 3u) | (A3 << 4u) |
                                (PARITY << 5u) | (1u << 7u));
  }

  static void SetBitLsb(uint8_t* buf, uint32_t bit_index, bool bit)
  {
    const uint32_t BYTE_INDEX = bit_index / 8u;
    const uint32_t BIT_OFFSET = bit_index & 7u;
    if (bit)
    {
      buf[BYTE_INDEX] = static_cast<uint8_t>(buf[BYTE_INDEX] | (1u << BIT_OFFSET));
    }
  }

  static bool GetBitLsb(const uint8_t* buf, uint32_t bit_index)
  {
    const uint32_t BYTE_INDEX = bit_index / 8u;
    const uint32_t BIT_OFFSET = bit_index & 7u;
    return ((buf[BYTE_INDEX] >> BIT_OFFSET) & 0x01u) != 0u;
  }

  static void EncodeMsbFirst(uint64_t payload, uint32_t bits, WireBuffer& packed)
  {
    packed.fill(0u);
    for (uint32_t i = 0u; i < bits; ++i)
    {
      const bool BIT = ((payload >> (bits - 1u - i)) & 0x01u) != 0u;
      SetBitLsb(packed.data(), i, BIT);
    }
  }

  static ErrorCode ReadMsbFirstU32(BitPort& port, uint32_t bits, uint32_t& out)
  {
    if (bits == 0u || bits > 32u)
    {
      return ErrorCode::ARG_ERR;
    }

    WireBuffer packed = {};
    const ErrorCode RESULT = port.SeqReadBits(bits, packed.data());
    if (RESULT != ErrorCode::OK)
    {
      return RESULT;
    }

    uint32_t value = 0u;
    for (uint32_t i = 0u; i < bits; ++i)
    {
      value = (value << 1u) | static_cast<uint32_t>(GetBitLsb(packed.data(), i));
    }
    out = value;
    return ErrorCode::OK;
  }

  static ErrorCode EncodeAndWriteMsbFirst(BitPort& port, uint64_t payload, uint32_t bits)
  {
    if (bits == 0u || bits > 64u)
    {
      return ErrorCode::ARG_ERR;
    }

    WireBuffer packed = {};
    EncodeMsbFirst(payload, bits, packed);
    return port.SeqWriteBits(bits, packed.data());
  }

  ErrorCode WriteWordRaw(uint8_t dmi_addr, uint32_t data)
  {
    const uint64_t FRAME = BuildWriteFrame(dmi_addr, data);

    debug_snapshot_.stage = 0x10u;
    debug_snapshot_.last_ec = static_cast<int8_t>(ErrorCode::OK);
    debug_snapshot_.last_req_addr = static_cast<uint8_t>(dmi_addr & DMI_ADDR_MASK);
    debug_snapshot_.last_req_tail = 0x02u;
    debug_snapshot_.last_req_data = data;
    debug_snapshot_.last_resp_addr = 0u;
    debug_snapshot_.last_resp_tail = 0u;
    debug_snapshot_.last_resp_data = 0u;
    debug_snapshot_.last_parity_rx = 0u;
    debug_snapshot_.last_parity_calc = 0u;
    debug_snapshot_.last_tx_lo = static_cast<uint32_t>(FRAME & 0xFFFF'FFFFu);
    debug_snapshot_.last_tx_hi = static_cast<uint32_t>(FRAME >> 32u);
    debug_snapshot_.last_rx_lo = 0u;
    debug_snapshot_.last_rx_hi = 0u;

    ErrorCode result = port_.BeginRvSwdFrame();
    if (result != ErrorCode::OK)
    {
      return result;
    }

    result = EncodeAndWriteMsbFirst(port_, FRAME, WRITE_FRAME_BITS);
    if (result != ErrorCode::OK)
    {
      entered_sdi_ = false;
      return result;
    }

    debug_snapshot_.stage = 0x11u;
    result = port_.EndRvSwdFrame();
    if (result == ErrorCode::OK)
    {
      debug_snapshot_.stage = 0x12u;
    }
    else
    {
      entered_sdi_ = false;
    }
    return result;
  }

  ErrorCode ReadWordRaw(uint8_t dmi_addr, uint32_t& data)
  {
    const uint16_t HEADER = BuildReadPrefix(dmi_addr);

    debug_snapshot_.stage = 0x20u;
    debug_snapshot_.last_ec = static_cast<int8_t>(ErrorCode::OK);
    debug_snapshot_.last_req_addr = static_cast<uint8_t>(dmi_addr & DMI_ADDR_MASK);
    debug_snapshot_.last_req_tail = 0x01u;
    debug_snapshot_.last_req_data = 0u;
    debug_snapshot_.last_resp_addr = static_cast<uint8_t>(dmi_addr & DMI_ADDR_MASK);
    debug_snapshot_.last_resp_tail = 0x00u;
    debug_snapshot_.last_resp_data = 0u;
    debug_snapshot_.last_parity_rx = 0u;
    debug_snapshot_.last_parity_calc = 0u;
    debug_snapshot_.last_tx_lo = HEADER;
    debug_snapshot_.last_tx_hi = 0u;
    debug_snapshot_.last_rx_lo = 0u;
    debug_snapshot_.last_rx_hi = 0u;

    ErrorCode result = port_.BeginRvSwdFrame();
    if (result != ErrorCode::OK)
    {
      return result;
    }

    result = EncodeAndWriteMsbFirst(port_, HEADER, READ_PREFIX_BITS);
    if (result != ErrorCode::OK)
    {
      entered_sdi_ = false;
      return result;
    }

    debug_snapshot_.stage = 0x21u;
    result = ReadMsbFirstU32(port_, DATA_BITS, data);
    if (result != ErrorCode::OK)
    {
      entered_sdi_ = false;
      return result;
    }

    std::array<uint8_t, 1u> parity_buf = {};
    result = port_.SeqReadBits(1u, parity_buf.data());
    if (result != ErrorCode::OK)
    {
      entered_sdi_ = false;
      return result;
    }
    const uint8_t PARITY_RX = GetBitLsb(parity_buf.data(), 0u) ? 1u : 0u;
    const uint8_t PARITY_CALC = Parity32(data);
    debug_snapshot_.last_parity_rx = PARITY_RX;
    debug_snapshot_.last_parity_calc = PARITY_CALC;

    debug_snapshot_.last_resp_data = data;
    debug_snapshot_.last_rx_lo = data;
    debug_snapshot_.stage = 0x22u;

    WireBuffer tail = {};
    EncodeMsbFirst(RVSWD_TAIL, RVSWD_HOST_TAIL_BITS, tail);
    result = port_.SeqWriteBits(RVSWD_HOST_TAIL_BITS, tail.data());
    if (result != ErrorCode::OK)
    {
      entered_sdi_ = false;
      return result;
    }

    debug_snapshot_.stage = 0x23u;
    result = port_.EndRvSwdFrame();
    if (result == ErrorCode::OK)
    {
      debug_snapshot_.stage = 0x24u;
    }
    else
    {
      entered_sdi_ = false;
    }
    if (PARITY_RX != PARITY_CALC)
    {
      return ErrorCode::FAILED;
    }
    return result;
  }

 private:
  BitPort& port_;
  bool entered_sdi_ = false;
};

}  // namespace LibXR::Debug
