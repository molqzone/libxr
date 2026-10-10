#include <array>
#include <cassert>
#include <cstring>
#include <cstdint>
#include <initializer_list>
#include <type_traits>
#include <utility>

#include "core/esc_registers.hpp"
#include "device/device_core.hpp"
#include "device/coe/coe_layout_device.hpp"

using namespace LibXR;
using namespace LibXR::EtherCAT;

namespace
{

struct WideStringLayout
{
  enum class DataType : uint16_t
  {
    BOOLEAN = 0x0001,
    INTEGER8 = 0x0002,
    INTEGER16 = 0x0003,
    INTEGER32 = 0x0004,
    UNSIGNED8 = 0x0005,
    UNSIGNED16 = 0x0006,
    UNSIGNED32 = 0x0007,
    REAL32 = 0x0008,
    VISIBLE_STRING = 0x0009,
    OCTET_STRING = 0x000A,
    REAL64 = 0x0011,
    INTEGER64 = 0x0015,
    UNSIGNED64 = 0x001B
  };
  enum class Direction : uint8_t
  {
    RX = 0,
    TX = 1
  };
  struct Entry
  {
    uint16_t index;
    uint8_t subindex;
    uint16_t bit_length;
    DataType type;
    uint16_t access;
    uint16_t pdo_index;
    Direction pdo_direction;
    const char* object_name;
  };
  struct Pdo
  {
    uint16_t index;
    Direction direction;
  };

  static constexpr Entry kEntries[] = {{0x2000, 1, 72, DataType::OCTET_STRING,
                                        0x78, 0x1600, Direction::RX, "Window"}};
  static constexpr Pdo kPdos[] = {{0x1600, Direction::RX}};
  static constexpr size_t kEntryCount = 1;
  static constexpr size_t kObjectCount = 1;
  static constexpr size_t kPdoCount = 1;
  static constexpr uint16_t ACCESS_RX = 0x78;
  static constexpr uint16_t ACCESS_TX = 0x87;
};

template <WideStringLayout::DataType Type, uint16_t Bits>
struct ScalarLayout : WideStringLayout
{
  inline static constexpr Entry kEntries[] = {
      {0x2000, 1, Bits, Type, ACCESS_RX, 0x1600, Direction::RX, "Value"}};
};

using BooleanLayout = ScalarLayout<WideStringLayout::DataType::BOOLEAN, 1>;

static_assert(CoeLayoutDevice<WideStringLayout>::kMaxStorageBytes == 9U);
static_assert(CoeLayoutDevice<BooleanLayout>::kMaxStorageBytes == 1U);
static_assert(ObjectDataTypeOf<bool>() == ObjectDataType::BOOLEAN);
static_assert(ObjectBitLengthOf<bool>() == 1U);
static_assert(std::is_same_v<
              decltype(std::declval<const ObjectDictionary&>().FindEntry(ObjectAddress{})),
              const ObjectEntry*>);
static_assert(std::is_same_v<
              decltype(std::declval<ObjectDictionary&>().FindEntry(ObjectAddress{})),
              ObjectEntry*>);

class FakeEsc final : public EscPort
{
 public:
  ErrorCode Read(uint16_t address, RawData data) override
  {
    if (data.addr_ == nullptr && data.size_ != 0U)
    {
      return ErrorCode::PTR_NULL;
    }
    if (static_cast<size_t>(address) + data.size_ > memory.size())
    {
      return ErrorCode::OUT_OF_RANGE;
    }
    std::memcpy(data.addr_, memory.data() + address, data.size_);
    return ErrorCode::OK;
  }

  ErrorCode Write(uint16_t address, ConstRawData data) override
  {
    if (data.addr_ == nullptr && data.size_ != 0U)
    {
      return ErrorCode::PTR_NULL;
    }
    if (static_cast<size_t>(address) + data.size_ > memory.size())
    {
      return ErrorCode::OUT_OF_RANGE;
    }
    std::memcpy(memory.data() + address, data.addr_, data.size_);
    return ErrorCode::OK;
  }

  std::array<uint8_t, 0x10000> memory{};
};

class TestMailboxProtocol final : public MailboxProtocol
{
 public:
  explicit TestMailboxProtocol(uint8_t protocol) : protocol_(protocol) {}

  uint8_t Protocol() const override { return protocol_; }
  void Handle(MailboxExchange&, const uint8_t*, size_t) override {}

 private:
  uint8_t protocol_;
};

void Put16(std::array<uint8_t, 0x10000>& memory, uint16_t address, uint16_t value)
{
  memory[address] = static_cast<uint8_t>(value);
  memory[address + 1U] = static_cast<uint8_t>(value >> 8U);
}

void Put32(std::array<uint8_t, 0x10000>& memory, uint16_t address, uint32_t value)
{
  Put16(memory, address, static_cast<uint16_t>(value));
  Put16(memory, static_cast<uint16_t>(address + 2U), static_cast<uint16_t>(value >> 16U));
}

/**
 * Model the ESC's request-buffer-full handshake. The master writing a mailbox
 * message is what sets the request SyncManager's status bit; the core reads a
 * *full* request buffer only (DeviceCore::ProcessMailbox: the SM event alone is
 * advisory, see the comment there) and clears the bit after reading. FakeEsc is
 * plain memory, so the test drives the bit by hand for every request the master
 * sends.
 */
void MasterWritesRequest(std::array<uint8_t, 0x10000>& memory)
{
  memory[0x0805] = EscRegister::SYNC_MANAGER_STATUS_MAILBOX;
}

class IoDevice final : public DeviceClass
{
 public:
  uint8_t output = 0;
  uint8_t input = 0;
  std::array<uint8_t, 8> parameter{};
  size_t object_write_count = 0;

 protected:
  void Describe(DeviceBuilder& builder) override
  {
    Object& output_object = builder.AddObject(0x6000, ObjectCode::VARIABLE, "Output");
    ObjectEntry& output_entry =
        builder.AddEntry(output_object, 0, ObjectDataType::UNSIGNED8, 8,
                         ObjectAccess::WRITE | ObjectAccess::RX_PDO, "Output", RawData(output));
    Pdo& rx = builder.AddPdo(PdoDirection::RX, 0x1600);
    builder.Map(rx, output_entry);

    Object& input_object = builder.AddObject(0x6010, ObjectCode::VARIABLE, "Input");
    ObjectEntry& input_entry =
        builder.AddEntry(input_object, 0, ObjectDataType::UNSIGNED8, 8,
                         ObjectAccess::READ | ObjectAccess::TX_PDO, "Input", RawData(input));
    Pdo& tx = builder.AddPdo(PdoDirection::TX, 0x1A00);
    builder.Map(tx, input_entry);

    Object& parameter_object = builder.AddObject(0x2000, ObjectCode::VARIABLE, "Parameter");
    builder.AddEntry(parameter_object, 0, ObjectDataType::OCTET_STRING, 64, ObjectAccess::READ_WRITE,
                     "Parameter", RawData(parameter.data(), parameter.size()));
  }

  ErrorCode OnObjectWrite(bool, ObjectEntry&) override
  {
    ++object_write_count;
    return ErrorCode::OK;
  }
};

}  // namespace

int main()
{
  CoeLayoutDevice<BooleanLayout> boolean_layout;
  bool* boolean_value = boolean_layout.BindAs<bool>(0x2000, 1);
  *boolean_value = true;
  assert(*boolean_value);

  FakeEsc esc;
  IoDevice application;
  StaticDevicePool<1, 3, 3, 2, 2, 8, 64> pool;
  esc.memory[0x0004] = 8;
  esc.memory[0x0005] = 8;

  Put16(esc.memory, 0x0800, 0x1000);
  Put16(esc.memory, 0x0802, 0);
  esc.memory[0x0804] = 0;
  Put16(esc.memory, 0x0808, 0x1000);
  Put16(esc.memory, 0x080A, 0);
  esc.memory[0x080C] = 0;

  Put16(esc.memory, 0x0820, 0x1000);
  Put16(esc.memory, 0x0822, 1);
  esc.memory[0x0824] = 0x04;
  Put16(esc.memory, 0x0828, 0x1100);
  Put16(esc.memory, 0x082A, 1);
  esc.memory[0x082C] = 0x00;

  Put32(esc.memory, 0x0600, 0x00000000);
  Put16(esc.memory, 0x0604, 1);
  Put16(esc.memory, 0x0608, 0x1000);
  esc.memory[0x060A] = 0x00;
  esc.memory[0x060B] = 0x02;
  esc.memory[0x060C] = 0x01;

  Put32(esc.memory, 0x0610, 0x00000000);
  Put16(esc.memory, 0x0614, 1);
  Put16(esc.memory, 0x0618, 0x1100);
  esc.memory[0x061A] = 0x00;
  esc.memory[0x061B] = 0x01;
  esc.memory[0x061C] = 0x01;

  DeviceCore core(esc, pool, {&application});
  TestMailboxProtocol error_protocol(MAILBOX_PROTOCOL_ERROR);
  assert(!core.RegisterMailboxProtocol(error_protocol));

  Put16(esc.memory, 0x0120, static_cast<uint16_t>(AlState::PRE_OPERATIONAL));
  core.HandleAlevent(EscRegister::EVENT_AL_CONTROL, false);
  assert(core.GetState() == AlState::PRE_OPERATIONAL);

  Put16(esc.memory, 0x0120, static_cast<uint16_t>(AlState::SAFE_OPERATIONAL));
  core.HandleAlevent(EscRegister::EVENT_AL_CONTROL | EscRegister::EVENT_SYNC_MANAGER_CHANGE, false);
  assert(core.GetState() == AlState::SAFE_OPERATIONAL);

  Put16(esc.memory, 0x0120, static_cast<uint16_t>(AlState::OPERATIONAL));
  core.HandleAlevent(EscRegister::EVENT_AL_CONTROL, false);
  assert(core.GetState() == AlState::OPERATIONAL);

  esc.memory[0x1000] = 0xA5;
  core.HandleAlevent(EscRegister::SyncManagerEvent(4), false);
  // Process data is polled, not moved by SM events: an SM event bit is not a
  // trustworthy "the master wrote outputs" signal on this hardware, see the
  // comment in DeviceCore::ProcessSyncManagerEvents.
  core.PollProcessData();
  assert(application.output == 0xA5);

  application.input = 0x5A;
  core.HandleAlevent(EscRegister::SyncManagerEvent(5), false);
  core.PollProcessData();
  assert(esc.memory[0x1100] == 0x5A);

  FakeEsc mailbox_esc;
  IoDevice mailbox_application;
  StaticDevicePool<1, 3, 3, 2, 2, 8, 64> mailbox_pool;
  mailbox_esc.memory[0x0004] = 8;
  mailbox_esc.memory[0x0005] = 8;
  Put16(mailbox_esc.memory, 0x0800, 0x1000);
  Put16(mailbox_esc.memory, 0x0802, 64);
  mailbox_esc.memory[0x0804] = 0x26;
  Put16(mailbox_esc.memory, 0x0808, 0x1040);
  Put16(mailbox_esc.memory, 0x080A, 64);
  mailbox_esc.memory[0x080C] = 0x22;

  DeviceCore mailbox_core(mailbox_esc, mailbox_pool, {&mailbox_application});
  Put16(mailbox_esc.memory, 0x0120, static_cast<uint16_t>(AlState::PRE_OPERATIONAL));
  mailbox_core.HandleAlevent(EscRegister::EVENT_AL_CONTROL, false);
  assert(mailbox_core.GetState() == AlState::PRE_OPERATIONAL);

  Put16(mailbox_esc.memory, 0x1000, 6);
  mailbox_esc.memory[0x1005] = 0x13;
  Put16(mailbox_esc.memory, 0x1006, 0x2000);
  mailbox_esc.memory[0x1008] = 0x40;
  Put16(mailbox_esc.memory, 0x1009, 0x6010);
  mailbox_esc.memory[0x100B] = 0;
  MasterWritesRequest(mailbox_esc.memory);
  mailbox_application.input = 0x31;
  mailbox_esc.memory[0x080D] = EscRegister::SYNC_MANAGER_STATUS_MAILBOX;
  mailbox_core.HandleAlevent(1U << 8U, false);
  assert(mailbox_esc.memory[0x1040] == 0);

  mailbox_esc.memory[0x080D] = 0;
  mailbox_core.HandleAlevent(EscRegister::SyncManagerEvent(1), false);
  assert(mailbox_esc.memory[0x1040] == 10);
  assert(mailbox_esc.memory[0x1048] == 0x4F);
  assert(mailbox_esc.memory[0x104C] == 0x31);

  Put16(mailbox_esc.memory, 0x1000, 10);
  mailbox_esc.memory[0x1005] = 0x23;
  Put16(mailbox_esc.memory, 0x1006, 0x2000);
  mailbox_esc.memory[0x1008] = 0x2F;
  Put16(mailbox_esc.memory, 0x1009, 0x6000);
  mailbox_esc.memory[0x100B] = 0;
  mailbox_esc.memory[0x100C] = 0x77;
  MasterWritesRequest(mailbox_esc.memory);
  mailbox_core.HandleAlevent(1U << 8U, false);
  assert(mailbox_application.output == 0x77);
  assert(mailbox_application.object_write_count == 1);
  assert(mailbox_esc.memory[0x1040] == 6);
  assert(mailbox_esc.memory[0x1048] == 0x60);

  mailbox_esc.memory[0x1040] = 0;
  // The master retransmits the request because the response never reached it,
  // so the request buffer is full again and the request is processed again.
  // There is no duplicate suppression beyond that, because the mailbox counter
  // cannot tell a retransmission from the next segment of one SDO transfer, see
  // the comment in DeviceCore::ProcessMailbox. The repeated write is idempotent
  // and the response is published again.
  MasterWritesRequest(mailbox_esc.memory);
  mailbox_core.HandleAlevent(EscRegister::SyncManagerEvent(0), false);
  assert(mailbox_application.object_write_count == 2);
  assert(mailbox_esc.memory[0x1040] == 6);
  assert(mailbox_esc.memory[0x1048] == 0x60);

  FakeEsc segmented_esc;
  IoDevice segmented_application;
  StaticDevicePool<1, 3, 3, 2, 2, 8, 16> segmented_pool;
  segmented_esc.memory[0x0004] = 8;
  segmented_esc.memory[0x0005] = 8;
  Put16(segmented_esc.memory, 0x0800, 0x1000);
  Put16(segmented_esc.memory, 0x0802, 16);
  segmented_esc.memory[0x0804] = 0x26;
  Put16(segmented_esc.memory, 0x0808, 0x1040);
  Put16(segmented_esc.memory, 0x080A, 16);
  segmented_esc.memory[0x080C] = 0x22;

  for (uint8_t index = 0; index < segmented_application.parameter.size(); ++index)
  {
    segmented_application.parameter[index] = static_cast<uint8_t>(index + 1U);
  }
  DeviceCore segmented_core(segmented_esc, segmented_pool, {&segmented_application});
  Put16(segmented_esc.memory, 0x0120, static_cast<uint16_t>(AlState::PRE_OPERATIONAL));
  segmented_core.HandleAlevent(EscRegister::EVENT_AL_CONTROL, false);

  Put16(segmented_esc.memory, 0x1000, 6);
  segmented_esc.memory[0x1005] = 0x13;
  Put16(segmented_esc.memory, 0x1006, 0x2000);
  segmented_esc.memory[0x1008] = 0x40;
  Put16(segmented_esc.memory, 0x1009, 0x2000);
  segmented_esc.memory[0x100B] = 0;
  MasterWritesRequest(segmented_esc.memory);
  segmented_core.HandleAlevent(1U << 8U, false);
  assert(segmented_esc.memory[0x1048] == 0x41);

  Put16(segmented_esc.memory, 0x1000, 3);
  segmented_esc.memory[0x1005] = 0x23;
  Put16(segmented_esc.memory, 0x1006, 0x2000);
  segmented_esc.memory[0x1008] = 0x60;
  MasterWritesRequest(segmented_esc.memory);
  segmented_core.HandleAlevent(1U << 8U, false);
  assert(segmented_esc.memory[0x1048] == 0x00);
  assert(segmented_esc.memory[0x1049] == 1);
  assert(segmented_esc.memory[0x104F] == 7);

  Put16(segmented_esc.memory, 0x1000, 3);
  segmented_esc.memory[0x1005] = 0x33;
  Put16(segmented_esc.memory, 0x1006, 0x2000);
  segmented_esc.memory[0x1008] = 0x70;
  MasterWritesRequest(segmented_esc.memory);
  segmented_core.HandleAlevent(1U << 8U, false);
  assert(segmented_esc.memory[0x1048] == 0x1D);
  assert(segmented_esc.memory[0x1049] == 8);
  assert(segmented_esc.memory[0x1040] == 4);

  segmented_application.parameter.fill(0);
  Put16(segmented_esc.memory, 0x1000, 10);
  segmented_esc.memory[0x1005] = 0x43;
  Put16(segmented_esc.memory, 0x1006, 0x2000);
  segmented_esc.memory[0x1008] = 0x21;
  Put16(segmented_esc.memory, 0x1009, 0x2000);
  segmented_esc.memory[0x100B] = 0;
  Put32(segmented_esc.memory, 0x100C, 8);
  MasterWritesRequest(segmented_esc.memory);
  segmented_core.HandleAlevent(1U << 8U, false);
  assert(segmented_esc.memory[0x1048] == 0x60);

  Put16(segmented_esc.memory, 0x1000, 10);
  segmented_esc.memory[0x1005] = 0x53;
  Put16(segmented_esc.memory, 0x1006, 0x2000);
  segmented_esc.memory[0x1008] = 0x00;
  for (uint8_t index = 0; index < 7; ++index)
  {
    segmented_esc.memory[0x1009 + index] = static_cast<uint8_t>(index + 1U);
  }
  MasterWritesRequest(segmented_esc.memory);
  segmented_core.HandleAlevent(1U << 8U, false);
  assert(segmented_esc.memory[0x1048] == 0x20);

  Put16(segmented_esc.memory, 0x1000, 10);
  segmented_esc.memory[0x1005] = 0x63;
  Put16(segmented_esc.memory, 0x1006, 0x2000);
  segmented_esc.memory[0x1008] = 0x1D;
  segmented_esc.memory[0x1009] = 8;
  MasterWritesRequest(segmented_esc.memory);
  segmented_core.HandleAlevent(1U << 8U, false);
  assert(segmented_esc.memory[0x1048] == 0x30);
  for (uint8_t index = 0; index < segmented_application.parameter.size(); ++index)
  {
    assert(segmented_application.parameter[index] == index + 1U);
  }
  return 0;
}
